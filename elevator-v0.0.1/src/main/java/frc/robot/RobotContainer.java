package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.elevator.ElevatorJoystickCommand;
import frc.robot.commands.elevator.ElevatorToHeightCommand;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkFlex;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Composition root: the ONLY place that knows which concrete classes are used.
 *
 * <p>It (1) picks the hardware implementation (real or simulated), (2) builds subsystems, and (3)
 * binds joystick inputs to commands. This is Dependency Inversion in practice: the subsystem only
 * knows the {@link ElevatorIO} interface.
 */
public class RobotContainer {
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);

  private final ElevatorSubsystem elevator =
      new ElevatorSubsystem(RobotBase.isReal() ? new ElevatorIOSparkFlex() : new ElevatorIOSim());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Logitech stick Y is NEGATIVE when pushed forward, so invert so "forward = up".
    var jog =
        new ElevatorJoystickCommand(
            elevator, () -> -joystick.getY(), this::speedScaleFromThrottle);

    // Moving the stick beyond the deadband takes control of the elevator. This also cancels any
    // running preset command (the scheduler interrupts it because both require the subsystem).
    // Releasing the stick ends the command; the subsystem keeps holding the last goal.
    new Trigger(() -> Math.abs(joystick.getY()) > OperatorConstants.JOYSTICK_DEADBAND)
        .whileTrue(jog);

    // Presets (buttons 3/4/5/6 are the top-of-stick buttons on the Extreme 3D Pro).
    joystick.button(OperatorConstants.BUTTON_STOW)
        .onTrue(new ElevatorToHeightCommand(elevator, ElevatorConstants.Presets.STOW_METERS));
    joystick.button(OperatorConstants.BUTTON_MID)
        .onTrue(new ElevatorToHeightCommand(elevator, ElevatorConstants.Presets.MID_METERS));
    joystick.button(OperatorConstants.BUTTON_HIGH)
        .onTrue(new ElevatorToHeightCommand(elevator, ElevatorConstants.Presets.HIGH_METERS));

    // Re-zero the encoder when the elevator is physically fully down. Works while disabled.
    joystick.button(OperatorConstants.BUTTON_ZERO)
        .onTrue(Commands.runOnce(elevator::zeroPosition).ignoringDisable(true));
  }

  /**
   * Maps the throttle slider to a speed multiplier in [MIN_SPEED_SCALE, 1]. On the Extreme 3D Pro
   * the slider reads -1 when pushed fully forward, so forward = fastest.
   */
  private double speedScaleFromThrottle() {
    double t = (1.0 - joystick.getThrottle()) / 2.0; // 0 (back) .. 1 (forward)
    return OperatorConstants.MIN_SPEED_SCALE + (1.0 - OperatorConstants.MIN_SPEED_SCALE) * t;
  }

  /** Called when any enabled mode starts. */
  public void onEnable() {
    // Avoid a sudden jump: the goal restarts at the current real position.
    elevator.holdCurrentPosition();
  }
}
