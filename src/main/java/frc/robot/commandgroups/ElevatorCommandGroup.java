// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCmd;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorCommandGroup extends SequentialCommandGroup {
  /** Creates a new ElevatorCommandGroup. */
  public ElevatorCommandGroup(ElevatorSubsystem subsystem, int level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
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
