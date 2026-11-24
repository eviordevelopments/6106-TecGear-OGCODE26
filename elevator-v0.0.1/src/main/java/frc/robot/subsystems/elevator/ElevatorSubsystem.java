package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

/**
 * Elevator control logic. Hardware-agnostic: it only uses {@link ElevatorIO}.
 *
 * <h2>Control strategy (simple and robust)</h2>
 *
 * <ol>
 *   <li>Everything is expressed as a <b>goal height</b> (meters). Commands only change the goal.
 *   <li>A {@link ProfiledPIDController} turns the goal into a smooth trapezoidal motion profile
 *       (limited speed and acceleration -> smooth starts AND controlled braking).
 *   <li>An {@link ElevatorFeedforward} adds kG (hold against gravity), kS (friction) and kV
 *       (speed). PID only corrects the leftover error, so tuning is easy.
 * </ol>
 *
 * <p>Joystick control = "move the goal at the joystick velocity" ({@link #jogGoal}). When the
 * stick is released the goal stops moving, the profile brakes smoothly and the PID + kG hold the
 * elevator at that spot. That gives: variable speed, stopping at any height, and continuing up or
 * down whenever you push again.
 *
 * <p>Single Responsibility: this class decides the motor voltage. Reading buttons lives in commands
 * and talking to motors lives in the IO classes.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorIO io;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ElevatorConstants.KP,
          ElevatorConstants.KI,
          ElevatorConstants.KD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.MAX_VELOCITY_MPS, ElevatorConstants.MAX_ACCELERATION_MPS2));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.KS_VOLTS,
          ElevatorConstants.KG_VOLTS,
          ElevatorConstants.KV_VOLTS_PER_MPS);

  /** Where we WANT the carriage to end up (meters). */
  private double goalMeters;

  private double lastAppliedVolts;

  /** @param io hardware (real or simulated) implementation. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    controller.setTolerance(
        ElevatorConstants.POSITION_TOLERANCE_METERS, ElevatorConstants.VELOCITY_TOLERANCE_MPS);
    holdCurrentPosition();
  }

  // ------------------------------------------------------------------------------------------
  // Public API used by commands
  // ------------------------------------------------------------------------------------------

  /** Sets an absolute goal height. It is clamped to the soft limits. */
  public void setGoalMeters(double meters) {
    goalMeters =
        MathUtil.clamp(
            meters, ElevatorConstants.MIN_HEIGHT_METERS, ElevatorConstants.MAX_HEIGHT_METERS);
  }

  /**
   * Moves the goal at {@code velocityMps} for one loop (call every cycle while the stick is held).
   *
   * @param velocityMps positive = up. Typically joystick * MAX_VELOCITY_MPS.
   */
  public void jogGoal(double velocityMps) {
    double next = goalMeters + velocityMps * LOOP_PERIOD_SECONDS;
    // Do not let the goal run far ahead of reality (e.g. mechanism jammed): avoids a lurch later.
    double position = io.getPositionMeters();
    next =
        MathUtil.clamp(
            next,
            position - ElevatorConstants.MAX_GOAL_LEAD_METERS,
            position + ElevatorConstants.MAX_GOAL_LEAD_METERS);
    setGoalMeters(next);
  }

  /** Makes the elevator hold where it is right now (goal = position, profile reset). */
  public void holdCurrentPosition() {
    double position = io.getPositionMeters();
    goalMeters = position;
    controller.reset(position, io.getVelocityMetersPerSecond());
  }

  /** Declares the current physical position as the bottom (use only when fully down). */
  public void zeroPosition() {
    io.resetPosition(ElevatorConstants.MIN_HEIGHT_METERS);
    holdCurrentPosition();
  }

  /** @return true when the carriage reached the goal and is (almost) stopped. */
  public boolean atGoal() {
    return controller.atGoal();
  }

  public double getPositionMeters() {
    return io.getPositionMeters();
  }

  public double getGoalMeters() {
    return goalMeters;
  }

  // ------------------------------------------------------------------------------------------
  // Periodic: the one place where voltage is computed
  // ------------------------------------------------------------------------------------------

  @Override
  public void periodic() {
    io.update();

    double position = io.getPositionMeters();
    double velocity = io.getVelocityMetersPerSecond();

    if (DriverStation.isDisabled()) {
      // Never drive while disabled; keep the controller synced so enabling does not jump.
      io.setVoltage(0.0);
      lastAppliedVolts = 0.0;
      goalMeters = position;
      controller.reset(position, velocity);
    } else {
      controller.setGoal(goalMeters);
      // PID corrects the error to the profile's current setpoint...
      double pidVolts = controller.calculate(position);
      // ...and feedforward supplies the rest: kG + kS + kV * profile velocity.
      double ffVolts = feedforward.calculate(controller.getSetpoint().velocity);
      lastAppliedVolts =
          MathUtil.clamp(
              pidVolts + ffVolts, -ElevatorConstants.MAX_VOLTAGE, ElevatorConstants.MAX_VOLTAGE);
      io.setVoltage(lastAppliedVolts);
    }

    publishTelemetry(position, velocity);
  }

  /** Values to graph in SmartDashboard/Glass while tuning. */
  private void publishTelemetry(double position, double velocity) {
    SmartDashboard.putNumber("Elevator/PositionIn", Units.metersToInches(position));
    SmartDashboard.putNumber("Elevator/GoalIn", Units.metersToInches(goalMeters));
    SmartDashboard.putNumber("Elevator/VelocityMps", velocity);
    SmartDashboard.putNumber(
        "Elevator/ProfileSetpointMps", controller.getSetpoint().velocity);
    SmartDashboard.putNumber("Elevator/AppliedVolts", lastAppliedVolts);
    SmartDashboard.putBoolean("Elevator/AtGoal", atGoal());
  }
}
