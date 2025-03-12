package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.subsystems.InterfaceSubsystem;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InterfaceActionCmd extends Command {

  private InterfaceSubsystem subsystem;
  private InterfaceExecuteMode loc;

  public InterfaceActionCmd(InterfaceSubsystem subsystem, InterfaceExecuteMode loc) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.loc = loc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("running work", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("running work", true);
    System.out.println("running work");
    subsystem.driveToLoc(loc);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.forceStopExecution();
    subsystem.drive_command.cancel();
  }

  //  // Returns true when the command should end.
  //  @Override
  //  public boolean isFinished() {
  //    return true;
  //  }
}
