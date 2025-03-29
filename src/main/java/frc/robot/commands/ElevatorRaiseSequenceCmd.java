package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FlipEleSubsystem;

/**
 * A command that enforces a strict sequence for raising the elevator: 1. Open centerer
 * (immediately) 2. Wait 0.75 seconds 3. Raise elevator
 *
 * <p>This command uses a SequentialCommandGroup to guarantee the timing.
 */
public class ElevatorRaiseSequenceCmd extends SequentialCommandGroup {

  /**
   * Creates a new ElevatorRaiseSequenceCmd.
   *
   * @param subsystem The FlipEleSubsystem to control
   */
  public ElevatorRaiseSequenceCmd(FlipEleSubsystem subsystem) {
    // First open the centerer and turn on scoring mode
    addCommands(
        new Command() {
          @Override
          public void initialize() {
            // Step 1: Open centerer immediately
            subsystem.centerer.set(false);

            // Enter scoring mode to block banner sensor checks
            subsystem.setInScoringMode(true);

            // If in holding state, ensure gripper stays closed
            if (subsystem.isInHoldingState()) {
              subsystem.closeGripper();
            }
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        },

        // Step 2: Wait 0.75 seconds
        Commands.waitSeconds(0.75), // Changed to 0.75 seconds

        // Step 3: Raise elevator
        new Command() {
          @Override
          public void initialize() {
            FlipEleSubsystem.raiseFirstStage();
          }

          @Override
          public boolean isFinished() {
            return true;
          }
        });
  }
}
