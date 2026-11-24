package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ElevatorConstants;

/**
 * Real hardware: 4x NEO Vortex on SPARK Flex controllers.
 *
 * <p>Topology: motor 0 is the LEADER. Motors 1..3 are configured as hardware followers, so we only
 * command and read the leader. The elevator is mechanically linked by the shared axis, so all
 * motors see the same motion.
 *
 * <p>The encoder conversion factors are set so the SPARK reports METERS and METERS/SECOND directly,
 * which keeps all unit math out of the subsystem.
 *
 * <p>Requires the REVLib vendor dependency (WPILib: "Manage Vendor Libraries" -> online install).
 */
public class ElevatorIOSparkFlex implements ElevatorIO {
  private final SparkFlex leader;
  private final RelativeEncoder encoder;

  public ElevatorIOSparkFlex() {
    int[] ids = ElevatorConstants.MOTOR_CAN_IDS;

    // ---- Leader ----
    leader = new SparkFlex(ids[0], MotorType.kBrushless);
    SparkFlexConfig leaderConfig = baseConfig();
    leaderConfig.inverted(ElevatorConstants.LEADER_INVERTED);
    // Encoder reports carriage meters (position) and meters/second (velocity = RPM based).
    leaderConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.METERS_PER_MOTOR_ROTATION)
        .velocityConversionFactor(ElevatorConstants.METERS_PER_MOTOR_ROTATION / 60.0);
    leader.configure(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = leader.getEncoder();
    encoder.setPosition(ElevatorConstants.MIN_HEIGHT_METERS); // assume starts fully down

    // ---- Followers ----
    for (int i = 1; i < ids.length; i++) {
      SparkFlex follower = new SparkFlex(ids[i], MotorType.kBrushless);
      SparkFlexConfig followerConfig = baseConfig();
      followerConfig.follow(leader, ElevatorConstants.FOLLOWER_INVERTED[i]);
      follower.configure(
          followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /** Settings shared by every motor (brake mode, current limit, voltage compensation). */
  private static SparkFlexConfig baseConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config
        .idleMode(IdleMode.kBrake) // brake = helps hold the load when output is zero
        .smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT_AMPS)
        .voltageCompensation(ElevatorConstants.MAX_VOLTAGE);
    return config;
  }

  @Override
  public double getPositionMeters() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocityMetersPerSecond() {
    return encoder.getVelocity();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(
        MathUtil.clamp(volts, -ElevatorConstants.MAX_VOLTAGE, ElevatorConstants.MAX_VOLTAGE));
  }

  @Override
  public void resetPosition(double meters) {
    encoder.setPosition(meters);
  }
}
