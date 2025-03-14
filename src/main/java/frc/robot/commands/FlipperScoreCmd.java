package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipperSubsystem;

public class FlipperScoreCmd extends Command {

  private FlipperSubsystem subsystem;
  private double flipperDelay;

  public FlipperScoreCmd(FlipperSubsystem subsystem, double flipperDelay) {
    this.subsystem = subsystem;
    this.flipperDelay = flipperDelay;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.flipperScore(flipperDelay);
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
