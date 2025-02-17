
package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Commands
import frc.robot.commands.FlipperExtendAndScoreCommand;
import frc.robot.commands.FlipperRetractCommand;
// Subsystems
import frc.robot.subsystems.FlipperSubsystem;

// Create a sequential command group for Level 4
public class Level2CommandGroup extends SequentialCommandGroup {
    public Level2CommandGroup(FlipperSubsystem flipperSubsystem) {
        addCommands(
                new FlipperExtendAndScoreCommand(flipperSubsystem),
                new FlipperRetractCommand(flipperSubsystem));
    }
}
