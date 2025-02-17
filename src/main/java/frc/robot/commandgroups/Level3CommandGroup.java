
package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Commands
import frc.robot.commands.ElevatorRaiseFirstStageCommand;
import frc.robot.commands.ElevatorLowerFirstStageCommand;
import frc.robot.commands.FlipperExtendAndScoreCommand;
import frc.robot.commands.FlipperRetractCommand;
// Subsystems
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;

// Create a sequential command group for Level 3
public class Level3CommandGroup extends SequentialCommandGroup {
    public Level3CommandGroup(ElevatorSubsystem elevatorSubsystem, FlipperSubsystem flipperSubsystem) {
        addCommands(
                new ElevatorRaiseFirstStageCommand(elevatorSubsystem),
                new FlipperExtendAndScoreCommand(flipperSubsystem),
                new FlipperRetractCommand(flipperSubsystem),
                new ElevatorLowerFirstStageCommand(elevatorSubsystem));
    }
}
