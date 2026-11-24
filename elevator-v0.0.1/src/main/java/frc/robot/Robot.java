package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Robot lifecycle class.
 *
 * <p>Responsibility: run the command scheduler and notify the container of mode changes. All robot
 * wiring lives in {@link RobotContainer}; this class stays tiny on purpose (KISS).
 */
public class Robot extends TimedRobot {
  private final RobotContainer container = new RobotContainer();

  @Override
  public void robotPeriodic() {
    // Runs every subsystem periodic() and every scheduled command.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    container.onEnable();
  }

  @Override
  public void autonomousInit() {
    container.onEnable();
  }

  @Override
  public void testInit() {
    container.onEnable();
  }
}
