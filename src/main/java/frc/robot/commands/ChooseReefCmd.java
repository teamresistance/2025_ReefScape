package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ChooseReefCmd extends Command {

  private InterfaceSubsystem interfaceSubsystem;
  private DriveSubsystem driveSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private FlipperSubsystem flipperSubsystem;

  private int level = -1;
  private int pos = -1;
  private int rl = -1;
  private boolean exec;

  /*
  ChooseReefCmd(subsystem, level, pos, rl, exec)
  - level is an int from 0-3 representing the targeted reef level. level 1 and 0 may have the same elevator height BUT level 0 drops
  the coral into the trough and level 1 puts it on the reef!!!! level 0 also works for recieving coral (maybe...)
  - pos is the int position around the reef from 0-5
  - rl is an int either 0 or 1, 0 representing left and 1 representing right
  - exec is a boolean as to if the robot will actually go to the selected position. if this is false it only updates the stored position
  variables and the robot wont do a thing. only change to true if you want the robot to move

  - if you do not want to change a value, set it to -1 in the constructor (ie if i only wanted to execute the code (by setting exec to true))
  - all my other values would be -1. example: new ChooseReefCmd(subsystem, -1, 4, -1, false) would set the position to 4 and change no other
  values in the subsystem. if you put new ChooseReefCmd(subsystem, 0, 4, 0, false), it would set the level and rl variables to 0!!!
  */
  public ChooseReefCmd(
      InterfaceSubsystem interfaceSubsystem,
      DriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      FlipperSubsystem flipperSubsystem,
      int level,
      int pos,
      int rl,
      boolean exec) {
    this.interfaceSubsystem = interfaceSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.flipperSubsystem = flipperSubsystem;
    if (level > -1) {
      this.level = level;
    }
    if (pos > -1) {
      this.pos = pos;
    }
    if (rl > -1) {
      this.rl = rl;
    }
    this.exec = exec;
    addRequirements(interfaceSubsystem, driveSubsystem, elevatorSubsystem, flipperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (exec) {
      interfaceSubsystem.chooseReef(
          driveSubsystem, elevatorSubsystem, flipperSubsystem, level, pos, rl);
    } else {
      interfaceSubsystem.chooseVars(level, pos, rl);
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
