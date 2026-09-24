# Control methods

This note records the three control methods considered for the linear inverted
pendulum. The deployed autonomous sequence currently uses **energy shaping for
swing-up** and **LQR for balance**. PID is retained as a transparent baseline
and fallback design, but it is not selected in the current firmware path.

## 1. PID / cascaded PD baseline

Near upright, use an inner pendulum-angle loop and an outer cart-position loop:

```text
theta_error = wrap(theta - pi)
position_correction = Kpx*x + Kdx*x_dot
u = -(Kpt*theta_error + Kdt*theta_dot + position_correction)
```

Integral action should normally be omitted or tightly clamped because rail
limits and motor saturation create wind-up. This controller is useful for sign
checks and manual tuning, but a single fixed gain set does not perform the full
swing-up from the downward equilibrium.

## 2. Energy-shaping swing-up

The pendulum energy estimate is

```text
E = 0.0032*theta_dot^2 + 0.1764*(cos(theta) - 1)
```

The deployed calculation follows the supplied Wheeltec firmware directly:

```text
if E >= 0: u = 0
else:
    accel = 98*sign(E*theta_dot*cos(theta))
            + sign(x)*log(1 - min(abs(x)/0.15, 0.95))
    u = 0.0651*accel + 11.5556*x_dot
        + 0.0028*cos(theta)*(0.1764*sin(theta) - 0.0180*accel)/0.0012
        - 0.8440*sin(theta)*theta_dot^2
```

Here `x` follows the supplied Wheeltec convention: positive is toward the
right. Because raw TIM4 ticks increase toward the left, the implementation uses
`x = -encoder_ticks` and `x_dot = -encoder_ticks_per_second`.

The implementation clamps the swing command and overrides it with a return
command when the cart leaves the central swing window. See
`lib/CartpoleControl/src/swingup_controller.c`.

Because the sign law is undefined at the exact downward, motionless equilibrium,
the relay is initialized to the positive direction for the first kick. A
`0.45 rad/s` dead band holds the previous relay sign while angular velocity
crosses zero, preventing ADC derivative noise from chattering the H-bridge.
The cart-return mode enters at `|x| > 1700` ticks and remains latched until
`|x| < 1100` ticks. Its inward command has a minimum magnitude so it can
overcome static friction; a cart already moving quickly inward is allowed to
coast. The loop runs every 5 ms. Swing-up permits a `0.1125` command change per
sample (the 22.5 /s rate originally tuned at 20 ms), while captured LQR uses a
separate `0.10` per-sample slew limit.

The supplied firmware's near-down expression evaluates to approximately
`10.50 V / 12 V = 0.875` before its `6900/7199` clamp. The hardware preset
therefore uses a `0.88` startup/pump command rather than the lower exploratory
setting used during initial bring-up.

## 3. LQR upright balance

Around upright, the state is

```text
state = [x, x_dot, theta_error, theta_dot]
u_voltage = -K * state
```

The current 40 cm reference gains are:

```text
K = [-36.4232, -40.0996, -101.4914, -15.8376]
```

The voltage command is divided by the 12 V supply and clamped to the PWM
authority limit. Encoder signs and the upright angle convention are converted
explicitly in `lib/CartpoleControl/src/lqr_controller.c`.

## Hybrid handoff

The controller starts in energy-shaping mode. It enters LQR after 10 ms with
`|theta_err| < 0.30 rad`, `|theta_dot| < 3.5 rad/s` and a predicted angle
`|theta_err + 0.2*theta_dot| < 0.25 rad`. The predicted-angle gate follows the
LQR recovery region: a pendulum crossing the top quickly is left to swing-up
instead of being grabbed by LQR, which would otherwise drive the cart into the
rail and lose it. Entry is fast so the swing-up relay cannot kick the pendulum
off the top while it waits. It exits LQR when `0.45 rad` or `7 rad/s` is
exceeded for 20 ms. A shared slew limiter bounds the command step using the
mode-specific slew limit at every 5 ms sample.

All numerical parameters are centralized in `include/cartpole_config.h`.
