package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ElevatorConstants;

/**
 * Physics simulation of the elevator (runs when the code is executed in the WPILib simulator).
 *
 * <p>Lets you test the controller, joystick mapping and tuning direction without a robot. It uses
 * the same {@link ElevatorConstants}, so a wrong constant shows up here too.
 */
public class ElevatorIOSim implements ElevatorIO {
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNeoVortex(ElevatorConstants.MOTOR_CAN_IDS.length),
          ElevatorConstants.TOTAL_REDUCTION,
          ElevatorConstants.SIM_CARRIAGE_MASS_KG,
          ElevatorConstants.EFFECTIVE_DRUM_RADIUS_METERS,
          ElevatorConstants.MIN_HEIGHT_METERS,
          ElevatorConstants.MAX_HEIGHT_METERS,
          true, // simulate gravity
          ElevatorConstants.MIN_HEIGHT_METERS);

  @Override
  public void update() {
    sim.update(LOOP_PERIOD_SECONDS);
  }

  @Override
  public double getPositionMeters() {
    return sim.getPositionMeters();
  }

  @Override
  public double getVelocityMetersPerSecond() {
    return sim.getVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(
        MathUtil.clamp(volts, -ElevatorConstants.MAX_VOLTAGE, ElevatorConstants.MAX_VOLTAGE));
  }

  @Override
  public void resetPosition(double meters) {
    sim.setState(meters, 0.0);
  }
}
