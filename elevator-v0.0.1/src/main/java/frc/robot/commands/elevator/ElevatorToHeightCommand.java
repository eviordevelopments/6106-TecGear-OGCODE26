package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * Goes to a fixed height (preset) with the subsystem's smooth profile and finishes when there.
 * After finishing, the subsystem keeps holding that height by itself.
 */
public class ElevatorToHeightCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final double targetMeters;

  public ElevatorToHeightCommand(ElevatorSubsystem elevator, double targetMeters) {
    this.elevator = elevator;
    this.targetMeters = targetMeters;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setGoalMeters(targetMeters);
  }

  @Override
  public boolean isFinished() {
    return elevator.atGoal();
  }
}
