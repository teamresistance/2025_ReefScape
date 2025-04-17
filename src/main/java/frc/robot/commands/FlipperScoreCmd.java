package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipEleSubsystem;

public class FlipperScoreCmd extends Command {

  private final FlipEleSubsystem subsystem;
  private final double flipperDelay;

  public FlipperScoreCmd(FlipEleSubsystem subsystem, double flipperDelay) {
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
    subsystem.flipperScore(flipperDelay, 0.5);
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
