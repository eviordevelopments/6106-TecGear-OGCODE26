package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Manual control: stick position = elevator speed.
 *
 * <p>Each cycle it moves the elevator GOAL by (stick * maxSpeed * dt). It never sets voltage, so
 * smoothing, braking and holding are all handled by the subsystem's profile + PID + kG.
 *
 * <p>The command receives suppliers instead of a joystick object, so it does not depend on any
 * specific controller (Dependency Inversion) and is easy to test.
 */
public class ElevatorJoystickCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final DoubleSupplier stickInput; // -1..1, positive = up
  private final DoubleSupplier speedScale; // 0..1 multiplier (throttle slider)

  public ElevatorJoystickCommand(
      ElevatorSubsystem elevator, DoubleSupplier stickInput, DoubleSupplier speedScale) {
    this.elevator = elevator;
    this.stickInput = stickInput;
    this.speedScale = speedScale;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    // Start from the real position so the goal does not jump when taking control.
    elevator.holdCurrentPosition();
  }

  @Override
  public void execute() {
    double input = MathUtil.applyDeadband(stickInput.getAsDouble(), OperatorConstants.JOYSTICK_DEADBAND);
    // Squaring keeps the sign but gives finer control near the center.
    input = Math.copySign(input * input, input);

    elevator.jogGoal(input * ElevatorConstants.MAX_VELOCITY_MPS * speedScale.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false; // ends when the trigger releases it (stick back to center)
  }
}
