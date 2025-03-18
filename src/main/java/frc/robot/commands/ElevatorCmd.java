package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FlipEleSubsystem;

public class ElevatorCmd extends Command {

  private final boolean state;
  private final int level;

  private final FlipEleSubsystem elevator;

  public ElevatorCmd(FlipEleSubsystem subsystem, int level, boolean state) {
    this.level = level;
    this.state = state;
    this.elevator = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level == 1 && state) {
      // Use the DIRECT sequence command that forces proper sequence and timing
      new DirectElevatorSequenceCmd(elevator).schedule();
    } else if (level == 1 && !state) {
      // Lower elevator and handle post-lowering in the subsystem
      FlipEleSubsystem.lowerFirstStage();
    } else if (level == 2 && state) {
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
    } else if (level == 2 && !state) {
      // Lower both stages
      FlipEleSubsystem.lowerSecondStage();
      FlipEleSubsystem.lowerFirstStage();
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
