package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FlipEleSubsystem;

/**
 * Command to toggle the elevator between stage 0 (lowered) and stage 2 (fully raised). Used for
 * testing the elevator and centerer mechanism.
 */
public class ElevatorToggleCmd extends Command {

  private final FlipEleSubsystem elevator;
  private static boolean elevated = false; // Static to remember state between commands

  /**
   * Creates a new ElevatorToggleCmd.
   *
   * @param elevator The FlipEleSubsystem to control
   */
  public ElevatorToggleCmd(FlipEleSubsystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevated) {
      // Lower both stages
      FlipEleSubsystem.lowerFirstStage();
      FlipEleSubsystem.lowerSecondStage();
      elevated = false;
    } else {
      // Use the DIRECT sequence command that forces proper sequence and timing
      new DirectElevatorSequenceCmd(elevator).schedule();
      // Second stage will be scheduled after the sequence completes
      CommandScheduler.getInstance()
          .schedule(
              Commands.waitSeconds(1.0) // Reduced to 1.0 seconds to match 0.75s elevator delay
                  .andThen(
                      () -> {
                        FlipEleSubsystem.raiseSecondStage();
                      }));
      elevated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
