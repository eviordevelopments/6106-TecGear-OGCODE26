package frc.robot.constants;

/** Driver-station / joystick constants. No logic, only numbers. */
public final class OperatorConstants {
  private OperatorConstants() {}

  /** USB port of the Logitech joystick in the Driver Station. */
  public static final int JOYSTICK_PORT = 0;

  /** Stick travel (0..1) ignored around the center to avoid drift. */
  public static final double JOYSTICK_DEADBAND = 0.08;

  /** Lowest speed multiplier when the throttle slider is fully back (0..1). */
  public static final double MIN_SPEED_SCALE = 0.25;

  // Button numbers as shown in the Driver Station (1 = trigger).
  public static final int BUTTON_STOW = 3;
  public static final int BUTTON_MID = 4;
  public static final int BUTTON_HIGH = 5;
  public static final int BUTTON_ZERO = 12;
}
