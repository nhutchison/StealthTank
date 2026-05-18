# StealthTank — RC Tank Drive Mixer

An Arduino sketch for the **Pro Micro (ATmega32U4)** that reads two PWM channels from a standard hobby RC receiver and mixes them into a **differential (tank) drive** output on two motor channels.

Point one transmitter stick forward — both tracks go forward. Push it sideways — the tracks counter-rotate. Anywhere in between is mixed proportionally using the "diamond mix" algorithm so you get smooth, intuitive control on a two-tracked vehicle from a normal aircraft/car-style transmitter.

## Why This Exists

This sketch was written to bridge the **Stealth** control system to **REV Robotics SPARK MAX** motor controllers. The SPARK MAX accepts hobby servo PWM as one of its input modes but has no built-in way to take a single-stick throttle/steering input from an RC receiver and turn it into a tank-drive pair — it expects each channel to already represent one side of the drivetrain. Stealth, on the other hand, emits the raw throttle and steering channels from the transmitter rather than a pre-mixed left/right pair.

The Pro Micro sits between them: it reads the two raw RC channels coming out of the Stealth-side receiver, runs the diamond-mix algorithm, and presents the SPARK MAX controllers with the already-mixed left/right servo PWM signals they need. Without this intermediate step there is no clean path from a Stealth transmitter to tank-drive output on a pair of SPARK MAX controllers.

## Features

- Reads two RC PWM inputs (throttle + steering, 1000–2000 µs)
- Mixes them with the diamond-mix algorithm for proportional tank steering
- Outputs two servo-style PWM channels (1000–2000 µs) suitable for ESCs, Sabertooth in R/C mode, Roboteq, or anything that speaks hobby servo PWM
- Non-blocking, interrupt-driven pulse capture (no `pulseIn()` stalls)
- 500 ms failsafe — motors stop if the RC signal is lost
- Configurable invert flags for each channel and motor
- Adjustable global speed limit for safe initial testing
- Optional debug output over USB serial

## Hardware

- Arduino Pro Micro (ATmega32U4, 5V/16MHz)
- Any RC receiver with standard PWM channel outputs
- Two motor controllers / ESCs that accept hobby servo PWM (1000–2000 µs)
- Common ground between receiver, Pro Micro, and motor controllers
- Regulated 5V supply for the Pro Micro (do not power from receiver rail unless you know it can handle it)

## Wiring

| Pro Micro pin | Connection |
|---|---|
| D2 (INT0) | RC receiver — throttle (forward/reverse) signal |
| D3 (INT1) | RC receiver — steering (left/right) signal |
| D9 | Left motor controller — signal in |
| D10 | Right motor controller — signal in |
| GND | Common ground with receiver and motor controllers |
| VCC / RAW | 5V supply |

Pins D2 and D3 are used because they are hardware-interrupt-capable on the ATmega32U4, which is essential for clean, non-blocking pulse measurement.

## Installation

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. In **Boards Manager**, install the AVR boards package if it isn't already.
3. Select **Tools → Board → Arduino Leonardo** (the Pro Micro uses the same ATmega32U4 core; alternatively install the SparkFun Pro Micro board package and select that).
4. Open `rc_tank_mixer.ino`.
5. Select the correct serial port and upload.

No external libraries are required — the sketch uses only the built-in `Servo` library.

## Configuration

All tunable values are at the top of the sketch:

```cpp
#define THROTTLE_PIN    2     // Interrupt-capable input pin
#define STEERING_PIN    3     // Interrupt-capable input pin
#define LEFT_OUT_PIN    9
#define RIGHT_OUT_PIN   10

#define INVERT_LEFT     0     // 1 if the left track runs backward
#define INVERT_RIGHT    0     // 1 if the right track runs backward
#define INVERT_THROTTLE 0     // 1 if forward stick gives reverse
#define INVERT_STEERING 0     // 1 if left stick gives right turn

const int RC_MIN        = 1000;   // RC pulse endpoints (microseconds)
const int RC_CENTER     = 1500;
const int RC_MAX        = 2000;
const int DEADZONE_US   = 30;     // Stick deadzone around center
const int MAX_SPEED_PCT = 100;    // Global speed limit, 0-100
const unsigned long FAILSAFE_MS = 500;
```

**Start with `MAX_SPEED_PCT` set to something tame (40–50) for first runs.**

## First-Run Checklist

1. With the motors disconnected, upload the sketch and open the Serial Monitor at 115200 baud.
2. With both sticks centered, you should see roughly `T=1500us S=1500us  L=0% R=0%`. If the resting values are far off, adjust `RC_MIN` / `RC_MAX` to your transmitter's actual endpoints.
3. Push the throttle stick forward. You should see L and R go positive together. If they go negative, set `INVERT_THROTTLE 1`.
4. Push the steering stick right. L should go positive, R negative (or vice versa for left). If reversed, set `INVERT_STEERING 1`.
5. Reconnect the motors with the vehicle elevated so the tracks can spin freely. Verify each track turns the expected direction. Use `INVERT_LEFT` / `INVERT_RIGHT` to correct.
6. Test on the ground at low `MAX_SPEED_PCT`, then raise it once you trust the behaviour.

## How the Mixing Works

The diamond-mix algorithm treats the joystick input as a point on a 2D plane, constrains it to a diamond shape (so that diagonal inputs don't exceed the per-track maximum), and projects that point onto two axes — one for the left track, one for the right. The result is a pair of speed values in the range −100 to +100 which are then mapped back to 1000–2000 µs pulses.

The implementation here is adapted from BigHappyDude's mixing function as used in [nhutchison/shadow_md_q85](https://github.com/nhutchison/shadow_md_q85), which itself draws on Declan Shanaghy's "[Using Diamond Coordinates to Power a Differential Drive](https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf)".

## Failsafe Behaviour

If either RC channel stops producing valid pulses for longer than `FAILSAFE_MS` (default 500 ms), both motor outputs are commanded to neutral (1500 µs) until valid signals return. This protects against receiver disconnects, transmitter power loss, and out-of-range conditions.

The sketch also validates each captured pulse against an 800–2200 µs sanity window in the ISR, so glitches and noise on the input lines are ignored rather than passed downstream.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Motors twitch or run at startup before RC is bound | ESC arming sequence — most ESCs need to see neutral first. The sketch already sends neutral for 1 second on boot; if you need longer, increase the `delay(1000)` in `setup()`. |
| Constant "FAILSAFE" message | Signal pin not wired to an interrupt-capable input, or receiver not outputting on that channel. |
| Both motors spin the wrong way | Flip `INVERT_THROTTLE`, or swap which is the throttle channel on the receiver. |
| Vehicle steers the wrong direction | Flip `INVERT_STEERING`. |
| One track is reversed | Flip `INVERT_LEFT` or `INVERT_RIGHT` for the affected side. |
| Sticks have a large dead area at center | Increase `DEADZONE_US`. If you've widened it past ~80, your transmitter trim probably needs adjusting instead. |
| Twitchy steering near center | Decrease `DEADZONE_US` (but don't set it to 0 — RC noise will jitter the output). |

## Credits

- Diamond-mix algorithm: BigHappyDude, via [nhutchison/shadow_md_q85](https://github.com/nhutchison/shadow_md_q85)
- Underlying math: Declan Shanaghy's JabberBot documentation

## License

Released for personal, hobby, and educational use. Distributed without warranty — you are responsible for the safe operation of your vehicle.
