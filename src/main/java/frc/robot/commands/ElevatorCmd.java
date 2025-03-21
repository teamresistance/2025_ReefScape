package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCmd extends Command {

  private boolean state;
  private int level;

  public ElevatorCmd(ElevatorSubsystem subsystem, int level, boolean state) {
    this.level = level;
    this.state = state;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level == 1 && state) {
      ElevatorSubsystem.raiseFirstStage();
    } else if (level == 1 && !state) {
      ElevatorSubsystem.lowerFirstStage();
    } else if (level == 2 && state) {
      ElevatorSubsystem.raiseSecondStage();
    } else if (level == 2 && !state) {
      ElevatorSubsystem.lowerSecondStage();
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
