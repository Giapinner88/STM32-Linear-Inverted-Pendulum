#!/usr/bin/env python3
"""Run the cart-pole controller on the laptop; the STM32 acquires and actuates.

Modes:
  --shadow   laptop computes the whole controller (swing-up, capture, LQR) but
             the board keeps control; compares both commands. No risk.
  (default)  laptop supplies the balance command once the board reports the
             pendulum captured; swing-up stays on the board.
  --nmpc     pure RTI-NMPC (nmpc/) in every phase. No supervisor and no LQR:
             a frame where the QP solver fails sends u = 0 and re-seeds.
  --blend    NMPC, but when it stops being real time (late solves, backlog, QP
             failures) control drops to LQR + energy shaping, and returns to
             NMPC after it has proven fast again.
             --nmpc and --blend need .venv-nmpc/bin/python (acados, CasADi).
  --full     laptop supplies the command in every phase while the automatic
             sequence is running (with a wider cart-return margin than the
             firmware, see cartpole_control.py). The board only calibrates, measures, slews
             the command, guards the rail and falls back to its own controller
             if the commands stop for 30 ms.

Start the automatic sequence with the USER key on the board after this
script is running (press USER again to stop). Ctrl+C also stops it.

  ~/.platformio/penv/bin/python tools/laptop_control.py --shadow --seconds 60
  ~/.platformio/penv/bin/python tools/laptop_control.py --full --seconds 90
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import time

import serial

import cartpole_control as cc
from serial_link import (MODE_DIRECT, MODE_FULL, MODE_IDLE, PacketParser,
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
    status is logged by name, e.g. 'qp_status=3 (MINSTEP)'."""
    events = []
    if policy.last_qp_status != 0:
        events.append(f"qp_status={policy.last_qp_status} "
                      f"({ACADOS_STATUS.get(policy.last_qp_status, '?')})")
    if policy.last_status not in (0, 2):
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
    group = p.add_mutually_exclusive_group()
    group.add_argument("--shadow", action="store_true", help="compare only, no control")
    group.add_argument("--full", action="store_true", help="laptop controls every phase")
    group.add_argument("--nmpc", action="store_true", help="pure NMPC, no fallback")
    group.add_argument("--blend", action="store_true", help="NMPC with LQR + energy-shaping fallback")
    p.add_argument("--out", help="CSV path (default: logs/laptop_<mode>_<time>.csv)")
    args = p.parse_args()

    mode_name = ("shadow" if args.shadow else "lqr" if args.full
                 else "nmpc" if args.nmpc else "blend" if args.blend else "balance")
    use_nmpc = args.nmpc or args.blend
    port = args.port or (list_candidate_ports() or [None])[0]
    if port is None:
        raise SystemExit("No /dev/ttyUSB* or /dev/ttyACM* port found")
    out = args.out or time.strftime(f"logs/laptop_{mode_name}_%Y%m%d_%H%M%S.csv")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    try_realtime()
    ser = serial.Serial(port, args.baud, timeout=0.002)
    ser.reset_input_buffer()
    parser = PacketParser()
    ctrl = cc.Hybrid()
    policy = guard = None
    nmpc_fallback_frames = 0
    if use_nmpc:
        import sys
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "nmpc"))
        from nmpc_policy import NmpcPolicy
        from realtime_guard import RealtimeGuard
        print("Building/loading the NMPC solver ...")
        policy = NmpcPolicy()
        guard = RealtimeGuard() if args.blend else None
    was_running = False
    sent_at: dict[int, float] = {}
    seq = 0
    rtts: list[float] = []
    packets = lost_pkts = cmd_sent = 0
    last_frame = None
    frames = {"running": 0, "captured": 0, "laptop": 0}
    cmp_err = {"swing-up": [], "captured": []}
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
                    backlog = len(batch) >= 3  # >=2 frames behind; 2 is normal USB clumping
                    stale = use_nmpc and len(batch) > 1 and t is not batch[-1]
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
                    if running != was_running:
                        ctrl.reset()
                        was_running = running

                    x_rel = t.x_pos - t.center_ticks
                    u_laptop = 0.0
                    nmpc_stepped = False
                    ctrl_label = "board" if args.shadow else ""
                    solver_status = qp_status = ""
                    event: list[str] = []
                    if policy is not None and not running:
                        policy.idle(x_rel, t.x_vel, t.theta, t.theta_vel)
                        nmpc_stepped = True
                        ctrl_label = "idle-warmup"
                        solver_status, qp_status, event = solver_report(policy)
                    if running:
                        if policy is None:
                            u_raw = ctrl.step(x_rel, t.x_vel, t.theta, t.theta_vel)
                            if args.full:
                                ctrl_label = "LQR+ES"
                        else:
                            # LQR + energy shaping tracks the state so --blend can take
                            # over at once; with --nmpc its output is never used.
                            u_fallback = ctrl.step(x_rel, t.x_vel, t.theta, t.theta_vel)
                            if guard is not None and guard.fallback:
                                nmpc_fallback_frames += 1
                                u_raw = u_fallback
                                ctrl_label = "LQR+ES"
                                policy.follow(u_raw)
                                if guard.tick_fallback():
                                    ms = policy.trial(x_rel, t.x_vel, t.theta, t.theta_vel)
                                    solver_status, qp_status, event = solver_report(policy)
                                    event.insert(0, f"trial solve {ms:.1f} ms")
                                    guard.record_trial(ms, backlog)
                                    if not guard.fallback:
                                        event.append(f"switch->NMPC: {guard.events[-1][2]}")
                            else:
                                u_raw, ok = policy.step(x_rel, t.x_vel, t.theta, t.theta_vel)
                                nmpc_stepped = ok
                                ctrl_label = "NMPC"
                                solver_status, qp_status, event = solver_report(policy)
                                if not ok:  # pure NMPC coasts for the frame, blend falls back
                                    u_raw = u_fallback if guard is not None else 0.0
                                    ctrl_label = "LQR+ES" if guard is not None else "NMPC(u=0)"
                                    event.append("NMPC failed: " + ("LQR+ES this frame"
                                                                    if guard is not None else "u=0 this frame"))
                                    policy.follow(u_raw)
                                if guard is not None:
                                    guard.record((time.perf_counter() - now) * 1000.0, backlog, ok)
                                if guard is not None and guard.fallback:
                                    policy.interrupt()
                                    event.append(f"switch->LQR+ES: {guard.events[-1][2]}")
                                    print(f"\n*** NMPC not real time ({guard.events[-1][2]}): "
                                          "switching to LQR + energy shaping ***", flush=True)
                        u_slewed = ctrl.apply_slew(u_raw)
                        frames["running"] += 1
                        frames["captured"] += captured
                        frames["laptop"] += bool(t.flags & FLAG_DIRECT_ACTIVE)
                        if args.shadow:
                            cmp_err["captured" if captured else "swing-up"].append(
                                abs(u_slewed - t.u))
                        u_laptop = u_raw

                    if args.shadow:
                        mode_byte, u_cmd = MODE_IDLE, 0.0
                    elif args.full or use_nmpc:
                        mode_byte, u_cmd = MODE_FULL, u_laptop
                    else:
                        mode_byte = MODE_DIRECT
                        u_cmd = cc.lqr_command(x_rel, t.x_vel, t.theta, t.theta_vel) if captured else 0.0

                    w.writerow([f"{now - t0:.4f}", t.frame, t.flags, f"{t.x_pos:.0f}",
                                t.center_ticks, f"{t.theta:.4f}", f"{t.theta_vel:.3f}",
                                f"{t.u:.4f}", f"{u_cmd:.4f}", t.cmd_seq,
                                "" if rtt is None else f"{rtt:.2f}",
                                ctrl_label, solver_status, qp_status, "; ".join(event),
                                0 if stale else 1])
                    if stale:  # newer telemetry is already waiting: no command for this one
                        continue
                    seq = (seq + 1) & 0xFF
                    sent_at[seq] = time.perf_counter()
                    ser.write(build_command(u_cmd, mode_byte, seq))
                    cmd_sent += 1
                    if nmpc_stepped:
                        policy.prepare()  # RTI preparation while waiting for the next frame

                    if now - t0 - last_print >= 1.0:
                        last_print = now - t0
                        err_deg = math.degrees(cc.theta_error(t.theta))
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
            print("The board falls back to its own controller after 30 ms without commands.")
        finally:
            try:
                ser.write(build_command(0.0, MODE_IDLE, (seq + 1) & 0xFF))
            except (serial.SerialException, OSError):
                pass  # link already gone
            finally:
                ser.close()

    print(f"\nDone: {packets} telemetry packets, {cmd_sent} commands sent, {lost_pkts} lost")
    if rtts:
        r = sorted(rtts)
        print(f"link round trip (includes up to one 5 ms frame): median {percentile(r, 0.5):.1f} ms, "
              f"p95 {percentile(r, 0.95):.1f} ms, max {r[-1]:.1f} ms, n={len(r)}")
    if frames["running"]:
        print(f"running frames {frames['running']}, captured {frames['captured']}, "
              f"laptop command applied in {frames['laptop']}")
    if policy is not None and policy.solve_ms:
        ms = sorted(policy.solve_ms)
        if policy.feedback_ms:
            fb = sorted(policy.feedback_ms)
            print(f"NMPC feedback phase (critical path) {len(fb)}: median {percentile(fb, 0.5):.2f} ms, "
                  f"p95 {percentile(fb, 0.95):.2f} ms, max {fb[-1]:.2f} ms")
        print(f"NMPC solves {len(ms)}: median {percentile(ms, 0.5):.2f} ms, p95 {percentile(ms, 0.95):.2f} ms, "
              f"max {ms[-1]:.2f} ms; QP failures {policy.failures}; frames on LQR + energy shaping {nmpc_fallback_frames}")
        for kind, frame, why in (guard.events if guard is not None else []):
            print(f"  supervisor: {kind} at telemetry #{frame} ({why})")
    for phase, errs in cmp_err.items():
        if errs:
            e = sorted(errs)
            print(f"shadow |u_laptop - u_board| while {phase}: n={len(e)}, "
                  f"median {percentile(e, 0.5):.3f}, p95 {percentile(e, 0.95):.3f}, max {e[-1]:.3f}")


if __name__ == "__main__":
    main()
