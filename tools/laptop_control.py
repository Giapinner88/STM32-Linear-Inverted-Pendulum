#!/usr/bin/env python3
"""Pure laptop NMPC; STM32 provides sensing, slew limiting and rail protection.

Press USER to start/stop. Lost commands stop the NMPC-only firmware motor.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time

import serial

from serial_link import theta_error
from serial_link import (MODE_FULL, MODE_IDLE, PacketParser,
                         build_command, list_candidate_ports)

FLAG_CAPTURED = 0x01
FLAG_RUNNING = 0x02
FLAG_RAIL_BLOCKED = 0x04
FLAG_DIRECT_ACTIVE = 0x08


# acados status codes, shared by the solver return value and the QP solver status.
ACADOS_STATUS = {0: "SUCCESS", 1: "FAILURE", 2: "MAXITER", 3: "MINSTEP", 4: "QP_FAILURE",
                 5: "READY", 6: "UNBOUNDED", 7: "TIMEOUT"}


def solver_report(policy):
    """(solver_status, qp_status, events) of the latest NMPC solve. Any non-zero QP
    status is logged by name, e.g. 'qp_status=3 (MINSTEP)'; a QP step that was undone
    and solved again with another barrier start is logged as 'QP retried xN'.
    solver_status 0 means the step was accepted (see NmpcPolicy._robust)."""
    events = []
    if policy.last_rescued:
        events.append("re-seeded from a rollout after a failed step")
    if policy.last_retries:
        ok = policy.last_status == 0
        events.append(f"QP retried x{policy.last_retries} -> {'ok' if ok else 'still failed'}")
    if policy.last_qp_status != 0:
        events.append(f"qp_status={policy.last_qp_status} "
                      f"({ACADOS_STATUS.get(policy.last_qp_status, '?')})")
    if policy.last_status != 0:
        events.append(f"solver_status={policy.last_status} "
                      f"({ACADOS_STATUS.get(policy.last_status, '?')})")
    return policy.last_status, policy.last_qp_status, events


def try_realtime() -> None:
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
        print("SCHED_FIFO enabled")
    except (AttributeError, PermissionError, OSError):
        print("SCHED_FIFO unavailable (needs privileges); using normal scheduling")


def percentile(sorted_values, q):
    return sorted_values[min(len(sorted_values) - 1, int(q * len(sorted_values)))]


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--nmpc", action="store_true", help=argparse.SUPPRESS)  # legacy alias
    p.add_argument("--out", help="CSV path (default: logs/laptop_<mode>_<time>.csv)")
    p.add_argument("--upright-reseed", action=argparse.BooleanOptionalAction, default=True,
                   help="NMPC: try an upright initial guess when a captured plan still swings away")
    p.add_argument("--plan-log", metavar="PATH",
                   help="--nmpc: record every NMPC plan (nmpc/plan_log.py) and save it to PATH.npz "
                        "after the run; only reads the solver. Runs on "
                        "the raw pot and the firmware's rate filter")
    args = p.parse_args()
    mode_name = "nmpc"
    port = args.port or (list_candidate_ports() or [None])[0]
    if port is None:
        raise SystemExit("No /dev/ttyUSB* or /dev/ttyACM* port found")
    out = args.out or time.strftime(f"logs/laptop_{mode_name}_%Y%m%d_%H%M%S.csv")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    try_realtime()
    ser = serial.Serial(port, args.baud, timeout=0.002)
    ser.reset_input_buffer()
    parser = PacketParser()
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "nmpc"))
    from nmpc_policy import NmpcPolicy
    print("Building/loading the NMPC solver ...")
    # One plan-log row per telemetry frame (200 Hz), with 10 s to spare.
    log = (dict(plan_instrumentation=True, plan_log_frames=int(args.seconds / 0.005) + 2000)
           if args.plan_log else {})
    policy = NmpcPolicy(upright_reseed=args.upright_reseed, **log)
    plan_log = policy.plan_log
    sent_at: dict[int, float] = {}
    seq = 0
    rtts: list[float] = []
    packets = lost_pkts = cmd_sent = 0
    last_frame = None
    frames = {"running": 0, "captured": 0, "laptop": 0}
    t0 = time.perf_counter()
    last_print = 0.0
    print(f"{port} @ {args.baud}, mode={mode_name}, {args.seconds:.0f}s -> {out}")
    print("Press USER on the board to start the sequence; press it again or Ctrl+C to stop.")

    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["host_s", "frame", "flags", "x_ticks", "center", "theta", "theta_vel",
                    "u_applied", "u_laptop", "cmd_seq", "rtt_ms",
                    "ctrl", "solver_status", "qp_status", "event", "sent"])
        try:
            while time.perf_counter() - t0 < args.seconds:
                batch = parser.feed(ser.read(256))
                for t in batch:
                    now = time.perf_counter()
                    stale = len(batch) > 1 and t is not batch[-1]
                    packets += 1
                    if last_frame is not None:
                        gap = (t.frame - last_frame - 1) & 0xFFFF
                        if gap <= 100:  # a bigger jump is stale buffered data, not loss
                            lost_pkts += gap
                    last_frame = t.frame
                    rtt = None
                    if t.cmd_seq in sent_at:
                        rtt = (now - sent_at.pop(t.cmd_seq)) * 1000.0
                        rtts.append(rtt)
                    sent_at = {s: ts for s, ts in sent_at.items() if now - ts < 1.0}

                    running = bool(t.flags & FLAG_RUNNING)
                    captured = bool(t.flags & FLAG_CAPTURED)
                    x_rel = t.x_pos - t.center_ticks
                    # Raw pot angle and the firmware's filtered angular velocity.
                    n_theta, n_rate = t.theta, t.theta_vel
                    u_laptop = 0.0
                    nmpc_stepped = False
                    ctrl_label = ""
                    solver_status = qp_status = ""
                    event: list[str] = []
                    if not running:
                        policy.idle(x_rel, t.x_vel, n_theta, n_rate)
                        nmpc_stepped = True
                        ctrl_label = "idle-warmup"
                        solver_status, qp_status, event = solver_report(policy)
                    if running:
                        u_raw, ok = policy.step(x_rel, t.x_vel, n_theta, n_rate)
                        nmpc_stepped = ok
                        ctrl_label = "NMPC"
                        solver_status, qp_status, event = solver_report(policy)
                        if not ok:
                            u_raw = policy.plan_command()
                            ctrl_label = "NMPC(plan)"
                            event.append("NMPC failed: following the last good plan")
                            policy.follow(u_raw)
                        frames["running"] += 1
                        frames["captured"] += captured
                        frames["laptop"] += bool(t.flags & FLAG_DIRECT_ACTIVE)
                        u_laptop = u_raw
                    mode_byte, u_cmd = MODE_FULL, u_laptop

                    w.writerow([f"{now - t0:.4f}", t.frame, t.flags, f"{t.x_pos:.0f}",
                                t.center_ticks, f"{t.theta:.4f}", f"{t.theta_vel:.3f}",
                                f"{t.u:.4f}", f"{u_cmd:.4f}", t.cmd_seq,
                                "" if rtt is None else f"{rtt:.2f}",
                                ctrl_label, solver_status, qp_status, "; ".join(event),
                                0 if stale else 1])
                    if stale:  # newer telemetry is already waiting: no command for this one
                        if plan_log is not None:
                            plan_log.mark_sent(u_cmd, False, t.frame)
                        continue
                    seq = (seq + 1) & 0xFF
                    sent_at[seq] = time.perf_counter()
                    ser.write(build_command(u_cmd, mode_byte, seq))
                    cmd_sent += 1
                    if plan_log is not None:
                        plan_log.mark_sent(u_cmd, True, t.frame)
                    if nmpc_stepped:
                        policy.prepare()  # RTI preparation while waiting for the next frame

                    if now - t0 - last_print >= 1.0:
                        last_print = now - t0
                        err_deg = math.degrees(theta_error(t.theta))
                        recent = sorted(rtts[-200:])
                        rtt_txt = f"{percentile(recent, 0.5):4.1f}" if recent else "  --"
                        state = ("CAPTURED" if captured else
                                 "swing-up" if running else "idle")
                        src = "LAPTOP" if t.flags & FLAG_DIRECT_ACTIVE else "board"
                        rail = " RAIL!" if t.flags & FLAG_RAIL_BLOCKED else ""
                        print(f"{last_print:5.0f}s {state:8s} ctrl={src:6s} "
                              f"angle={err_deg:+7.1f}deg x={x_rel:+6.0f}tk "
                              f"u_board={t.u:+.2f} u_laptop={u_laptop:+.2f} "
                              f"rtt={rtt_txt}ms lost={lost_pkts}{rail}", flush=True)
        except KeyboardInterrupt:
            pass
        except (serial.SerialException, OSError) as exc:
            print(f"\nSERIAL LINK LOST after {time.perf_counter() - t0:.2f}s: {exc}")
            print("NMPC-only firmware stops the motor after the command timeout.")
        finally:
            try:
                ser.write(build_command(0.0, MODE_IDLE, (seq + 1) & 0xFF))
            except (serial.SerialException, OSError):
                pass  # link already gone
            finally:
                ser.close()

    if plan_log is not None:
        path = plan_log.save(args.plan_log, extra=dict(source="laptop_control", csv=out))
        print(f"plan log: {plan_log.n} frames ({plan_log.dropped} dropped) -> {path}")
    print(f"\nDone: {packets} telemetry packets, {cmd_sent} commands sent, {lost_pkts} lost")
    if rtts:
        r = sorted(rtts)
        print(f"link round trip (includes up to one 5 ms frame): median {percentile(r, 0.5):.1f} ms, "
              f"p95 {percentile(r, 0.95):.1f} ms, max {r[-1]:.1f} ms, n={len(r)}")
    if frames["running"]:
        print(f"running frames {frames['running']}, captured {frames['captured']}, "
              f"laptop command applied in {frames['laptop']}")
    if policy.solve_ms:
        ms = sorted(policy.solve_ms)
        if policy.feedback_ms:
            fb = sorted(policy.feedback_ms)
            print(f"NMPC feedback phase (critical path) {len(fb)}: median {percentile(fb, 0.5):.2f} ms, "
                  f"p95 {percentile(fb, 0.95):.2f} ms, max {fb[-1]:.2f} ms")
        print(f"NMPC solves {len(ms)}: median {percentile(ms, 0.5):.2f} ms, p95 {percentile(ms, 0.95):.2f} ms, "
              f"max {ms[-1]:.2f} ms; QP retries {policy.retries}, re-seeds {policy.reseeds} (state jumps {policy.jumps}, in-frame rescues {policy.rescues}), QP failures left {policy.failures} ({policy.overruns} at the 3 ms deadline)")
        print(f"NMPC upright plan repair: {policy.upright_accepts}/{policy.upright_attempts} accepted")


if __name__ == "__main__":
    main()
