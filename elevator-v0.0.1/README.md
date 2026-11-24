# FRC Cascade Elevator — v0.0.1

Java / WPILib command-based code for a 2-stage cascade elevator
(base 24 in, stage 1 22 in, stage 2 27 in; 4x NEO Vortex, 25:1, #35 chain, rope + pulleys).

## Folder layout (copy `src/` into a WPILib "Command Robot / Timed" project)

```
src/main/java/frc/robot/
├── Main.java                      entry point (do not edit)
├── Robot.java                     lifecycle + scheduler only
├── RobotContainer.java            composition root: picks real/sim IO, builds subsystem, binds buttons
├── constants/
│   ├── ElevatorConstants.java     ALL physical + tuning numbers (edit this first)
│   └── OperatorConstants.java     joystick port, deadband, button numbers
├── subsystems/elevator/
│   ├── ElevatorIO.java            hardware interface (small, 5 methods)
│   ├── ElevatorIOSparkFlex.java   real NEO Vortex implementation (1 leader + 3 followers)
│   ├── ElevatorIOSim.java         physics simulation
│   └── ElevatorSubsystem.java     control: profile + PID + kG/kS/kV, goal-based
└── commands/elevator/
    ├── ElevatorJoystickCommand.java   stick -> goal velocity
    └── ElevatorToHeightCommand.java   go to preset height
```

## How it works (one paragraph)
Everything sets a **goal height**. The subsystem follows it with a trapezoidal profile
(max speed/accel -> smooth start and controlled braking), feedforward (kG, kS, kV) and a PID for
the residual error. The joystick just moves the goal (`stick * maxSpeed * dt`); release it and the
elevator brakes and holds that height; push again to continue up or down.

## Setup
1. Install **REVLib** (WPILib VS Code: *Manage Vendor Libraries → Install new → online*). Use the
   current-season WPILib; the code uses the 2025+ REVLib API (`SparkFlex`, `SparkFlexConfig`).
2. Edit `ElevatorConstants`: CAN IDs, inversions, drum diameter, chain ratio, rigging multiplier
   (`CARRIAGE_TRAVEL_PER_ROPE_TRAVEL`), `MAX_HEIGHT_METERS`, presets.
3. Joystick: Driver Station port 0. Stick forward = up, throttle slider forward = full speed,
   buttons 3/4/5 = stow/mid/high presets, button 12 = re-zero (only when fully down).
4. Robot must start **fully down** (encoder is zeroed at boot).

## Commissioning / tuning order (robot on blocks, be ready to disable)
1. **Directions:** with the elevator unloaded, push stick up slowly. It must go UP; if not, flip
   `LEADER_INVERTED`; if motors fight each other, flip `FOLLOWER_INVERTED` entries.
2. **Units:** move it 12 in by hand and check `Elevator/PositionIn`. Fix `DRUM_DIAMETER`,
   `CHAIN_REDUCTION`, `CARRIAGE_TRAVEL_PER_ROPE_TRAVEL` until it matches the tape.
3. **kG:** raise `KG_VOLTS` until the carriage barely holds in the middle without drifting.
4. **kS / kV:** start with the theoretical kV; if cruising speed is below the profile speed, raise kV.
5. **kP then kD:** raise `KP` until it tracks well, add `KD` to remove overshoot when braking. Keep
   `KI = 0`.
6. Raise `MAX_VELOCITY_MPS` / `MAX_ACCELERATION_MPS2` step by step; lower the current limit
   if breakers or the rope are stressed.
7. Set real soft limits (`MAX_HEIGHT_METERS`) a little before the mechanical hard stops.

## Known limits of v0.0.1 (ideas for v0.0.2)
- No limit switch / absolute homing (manual re-zero button).
- No SysId characterization routine; no stall detection.
- Telemetry via SmartDashboard only.
