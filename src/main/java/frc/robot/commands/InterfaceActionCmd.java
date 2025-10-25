package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.subsystems.InterfaceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InterfaceActionCmd extends Command {

  private final InterfaceSubsystem subsystem;
  private final InterfaceExecuteMode loc;
  private boolean finished = true;
  private final int level;
  private final boolean right;

  public InterfaceActionCmd(
      InterfaceSubsystem subsystem, InterfaceExecuteMode loc, int level, boolean right) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.loc = loc;
    this.level = level;
    this.right = right;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //    Logger.recordOutput("running work", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //    Logger.recordOutput("finished work", false);
    subsystem.driveToLoc(loc, level, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //    subsystem.forceStopExecution();
    //    subsystem.drive_command.cancel();
    //    finished = true;
  }

  public void finishparentCommand() {
    finished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //    Logger.recordOutput("auto/state", finished);
    return true;
  }
}
