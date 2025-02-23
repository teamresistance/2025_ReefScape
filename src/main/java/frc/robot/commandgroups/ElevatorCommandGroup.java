package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommandGroup extends SequentialCommandGroup {
  /** Command group for elevator position (0, 1, 2 for low, mid, high) */
  public ElevatorCommandGroup(ElevatorSubsystem subsystem, int level) {
    if (level == 0) {
      addCommands(new ElevatorCmd(subsystem, 1, false), new ElevatorCmd(subsystem, 2, false));
    }
    if (level == 1) {
      addCommands(new ElevatorCmd(subsystem, 0, true), new ElevatorCmd(subsystem, 0, false));
    }
    if (level == 2) {
      addCommands(new ElevatorCmd(subsystem, 1, true), new ElevatorCmd(subsystem, 2, true));
    }
  }
}
