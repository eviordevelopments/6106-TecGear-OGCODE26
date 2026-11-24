package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * Every physical and tuning number of the elevator lives here, so adapting the code to the real
 * robot means editing ONE file. Values marked TODO are placeholders to be measured/tuned.
 *
 * <p>Unit convention: SI inside the code (meters, m/s, volts). Inch values are converted with
 * {@link Units#inchesToMeters(double)} so you can type the numbers you measure with a tape.
 */
public final class ElevatorConstants {
  private ElevatorConstants() {}

  // ---------------------------------------------------------------------------------------------
  // CAN / motors
  // ---------------------------------------------------------------------------------------------
  /**
   * CAN IDs of the four NEO Vortex (SPARK Flex). Index 0 is the LEADER (the only one we command
   * and read); the others follow it in hardware. TODO: set real IDs.
   */
  public static final int[] MOTOR_CAN_IDS = {10, 11, 12, 13};

  /** Invert the leader if positive voltage moves the elevator DOWN. TODO: verify on the robot. */
  public static final boolean LEADER_INVERTED = false;

  /**
   * Per-follower inversion (indexes 1..3 of MOTOR_CAN_IDS; index 0 is unused). Set true for the
   * motors on the side that spins the opposite way. TODO: verify on the robot.
   */
  public static final boolean[] FOLLOWER_INVERTED = {false, true, false, true};

  /** Per-motor current limit in amps. Protects breakers and the mechanism. TODO: tune. */
  public static final int CURRENT_LIMIT_AMPS = 50;

  /** NEO Vortex free speed (datasheet), used only to compute theoretical limits below. */
  public static final double MOTOR_FREE_SPEED_RPM = 6784.0;

  // ---------------------------------------------------------------------------------------------
  // Mechanism geometry (documentation + math)
  // ---------------------------------------------------------------------------------------------
  /** Section lengths, kept for reference/CAD checks (not used in the math). */
  public static final double BASE_LENGTH_METERS = Units.inchesToMeters(24.0);

  public static final double STAGE_1_LENGTH_METERS = Units.inchesToMeters(22.0);
  public static final double STAGE_2_LENGTH_METERS = Units.inchesToMeters(27.0);

  /** Planetary/gearbox reduction on each motor (25:1). */
  public static final double GEARBOX_REDUCTION = 25.0;

  /**
   * Extra reduction of the #35 chain between gearbox output and the rope drum axis
   * (driven teeth / driving teeth). 1.0 = same sprocket on both ends. TODO: set from real teeth.
   */
  public static final double CHAIN_REDUCTION = 1.0;

  /** Motor rotations per drum rotation. */
  public static final double TOTAL_REDUCTION = GEARBOX_REDUCTION * CHAIN_REDUCTION;

  /** Effective diameter of the rope drum (include rope thickness). TODO: measure. */
  public static final double DRUM_DIAMETER_METERS = Units.inchesToMeters(2.0);

  /**
   * How many meters the CARRIAGE moves for each meter of rope pulled by the drum.
   *
   * <p>With continuous rigging on a 2-moving-stage cascade, every stage adds its travel on top of
   * the previous one, so the carriage moves 2x. Use 1.0 if your rigging is 1:1 (carriage = stage
   * speed). TODO: confirm with the real rigging.
   */
  public static final double CARRIAGE_TRAVEL_PER_ROPE_TRAVEL = 2.0;

  /** Carriage meters for ONE motor rotation. Used as the encoder conversion factor. */
  public static final double METERS_PER_MOTOR_ROTATION =
      (Math.PI * DRUM_DIAMETER_METERS / TOTAL_REDUCTION) * CARRIAGE_TRAVEL_PER_ROPE_TRAVEL;

  /** Equivalent drum radius at the carriage (needed by the physics simulation). */
  public static final double EFFECTIVE_DRUM_RADIUS_METERS =
      METERS_PER_MOTOR_ROTATION * TOTAL_REDUCTION / (2.0 * Math.PI);

  // ---------------------------------------------------------------------------------------------
  // Travel limits (software limits)
  // ---------------------------------------------------------------------------------------------
  /** Height with the elevator fully down. The encoder is zeroed here at boot. */
  public static final double MIN_HEIGHT_METERS = 0.0;

  /** Total carriage travel from fully down to fully up. TODO: measure on the real robot. */
  public static final double MAX_HEIGHT_METERS = Units.inchesToMeters(40.0);

  /** Presets reachable with joystick buttons. TODO: set real scoring heights. */
  public static final class Presets {
    private Presets() {}

    public static final double STOW_METERS = MIN_HEIGHT_METERS;
    public static final double MID_METERS = Units.inchesToMeters(20.0);
    public static final double HIGH_METERS = MAX_HEIGHT_METERS;
  }

  // ---------------------------------------------------------------------------------------------
  // Motion limits (how smooth / how aggressive)
  // ---------------------------------------------------------------------------------------------
  /** Theoretical top speed of the carriage at 12 V (no load). */
  public static final double FREE_SPEED_METERS_PER_SEC =
      (MOTOR_FREE_SPEED_RPM / 60.0) * METERS_PER_MOTOR_ROTATION;

  /** Max commanded speed (also the joystick's full-scale speed). TODO: raise slowly. */
  public static final double MAX_VELOCITY_MPS = 1.0;

  /** Max accel/decel. Higher = snappier start AND braking. TODO: raise slowly. */
  public static final double MAX_ACCELERATION_MPS2 = 2.0;

  /**
   * Max distance the goal may run ahead of the real position while the joystick is held. Prevents
   * a huge "stored" error if the mechanism is blocked, so it never lurches after being freed.
   */
  public static final double MAX_GOAL_LEAD_METERS = Units.inchesToMeters(6.0);

  // ---------------------------------------------------------------------------------------------
  // Control gains (TODO: tune on the real robot, see README)
  // ---------------------------------------------------------------------------------------------
  /** Gravity feedforward: volts that hold the carriage still. Measure first! */
  public static final double KG_VOLTS = 0.40;

  /** Static friction feedforward: volts to just start moving. */
  public static final double KS_VOLTS = 0.10;

  /** Velocity feedforward in V per (m/s). Theoretical value; refine with SysId if needed. */
  public static final double KV_VOLTS_PER_MPS = 12.0 / FREE_SPEED_METERS_PER_SEC;

  /** Proportional gain in V per meter of position error. */
  public static final double KP = 20.0;

  /** Integral gain. Keep 0 unless there is steady-state error (rare with a good kG). */
  public static final double KI = 0.0;

  /** Derivative gain in V per (m/s) of error change; damps overshoot while braking. */
  public static final double KD = 0.5;

  /** "Arrived" tolerances used by preset commands. */
  public static final double POSITION_TOLERANCE_METERS = Units.inchesToMeters(0.4);

  public static final double VELOCITY_TOLERANCE_MPS = 0.05;

  /** Absolute voltage cap sent to the motors (battery compensation is 12 V). */
  public static final double MAX_VOLTAGE = 12.0;

  // Simulation only
  public static final double SIM_CARRIAGE_MASS_KG = 6.0;
}
