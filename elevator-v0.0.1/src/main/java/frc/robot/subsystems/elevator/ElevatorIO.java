package frc.robot.subsystems.elevator;

/**
 * Hardware abstraction of the elevator (Dependency Inversion / Interface Segregation).
 *
 * <p>{@link ElevatorSubsystem} talks ONLY to this interface, so the same control code runs on the
 * real robot ({@link ElevatorIOSparkFlex}) and in simulation ({@link ElevatorIOSim}). Adding a new
 * sensor or motor controller means a new implementation, not changing the subsystem (Open/Closed).
 *
 * <p>All values are SI: meters, meters per second, volts.
 */
public interface ElevatorIO {
  /** Called once per loop BEFORE reading values (lets simulations advance their physics). */
  default void update() {}

  /** Carriage height above the zero point, in meters. */
  double getPositionMeters();

  /** Carriage velocity in meters per second (positive = up). */
  double getVelocityMetersPerSecond();

  /** Applies voltage to the motors (positive = up). Implementations must clamp to safe range. */
  void setVoltage(double volts);

  /** Declares the current physical position as {@code meters} (used for zeroing). */
  void resetPosition(double meters);
}
