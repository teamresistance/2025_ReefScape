
package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// Commands
import frc.robot.commands.FlipperExtendAndScoreCommand;
import frc.robot.commands.FlipperRetractCommand;
import frc.robot.commands.ElevatorRaiseSecondStageCommand;
import frc.robot.commands.ElevatorLowerSecondStageCommand;
import frc.robot.commands.ElevatorRaiseFirstStageCommand;
import frc.robot.commands.ElevatorLowerFirstStageCommand;
// Subsystems
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;

// Create a sequential command group for Level 4
public class Level4CommandGroup extends SequentialCommandGroup {

    public class RaiseBothStagesCommandGroup extends ParallelCommandGroup {
        public RaiseBothStagesCommandGroup(ElevatorSubsystem elevatorSubsystem) {
            addCommands(
                    new ElevatorRaiseFirstStageCommand(elevatorSubsystem),
                    new ElevatorRaiseSecondStageCommand(elevatorSubsystem));
        }
    }

    public class LowerBothStagesCommandGroup extends ParallelCommandGroup {
        public LowerBothStagesCommandGroup(ElevatorSubsystem elevatorSubsystem) {
            addCommands(
                    new ElevatorLowerSecondStageCommand(elevatorSubsystem),
                    new ElevatorLowerFirstStageCommand(elevatorSubsystem));
        }
    }

    public Level4CommandGroup(ElevatorSubsystem elevatorSubsystem, FlipperSubsystem flipperSubsystem) {
        addCommands(
                new RaiseBothStagesCommandGroup(elevatorSubsystem),
                new FlipperExtendAndScoreCommand(flipperSubsystem),
                new FlipperRetractCommand(flipperSubsystem),
                new LowerBothStagesCommandGroup(elevatorSubsystem));
    }
}
