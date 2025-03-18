package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipEleSubsystem;

public class ElevatorCmd extends Command {

  private boolean state;
  private int level;

  private FlipEleSubsystem elevator;

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
    // System.out.println("Execute");
    if (level == 1 && state) {
      // System.out.println("state 1");
      FlipEleSubsystem.raiseFirstStage();
    } else if (level == 1 && !state) {
      FlipEleSubsystem.lowerFirstStage();
    } else if (level == 2 && state) {
      FlipEleSubsystem.raiseSecondStage();
      FlipEleSubsystem.raiseFirstStage();
    } else if (level == 2 && !state) {
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
