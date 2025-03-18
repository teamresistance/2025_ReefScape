package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipEleSubsystem;

/**
 * A direct command that forces the correct sequence with hard timing: 1. Open centerer 2. Wait 0.75
 * seconds 3. Raise elevator
 *
 * <p>This command does not rely on scheduling other commands and directly controls the timing
 * within a single command execution.
 */
public class DirectElevatorSequenceCmd extends Command {

  private final FlipEleSubsystem subsystem;
  private final Timer delayTimer = new Timer();
  private boolean centererOpened = false;
  private boolean elevatorRaised = false;

  public DirectElevatorSequenceCmd(FlipEleSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // Reset everything at start
    centererOpened = false;
    elevatorRaised = false;
    delayTimer.reset();
    delayTimer.start();

    // Step 1: Immediately open centerer
    subsystem.openCenterer();
    centererOpened = true;

    // Enter scoring mode to block banner sensors
    subsystem.setInScoringMode(true);

    // If in holding state, keep the gripper closed
    if (subsystem.isInHoldingState()) {
      subsystem.closeGripper();
    }
  }

  @Override
  public void execute() {
    // Always make sure centerer stays open during this sequence
    subsystem.openCenterer();

    // Check if delay has elapsed and we haven't raised elevator yet
    if (centererOpened
        && !elevatorRaised
        && delayTimer.hasElapsed(0.75)) { // Changed to 0.75 seconds
      // Step 3: Now raise the elevator
      FlipEleSubsystem.raiseFirstStage();
      elevatorRaised = true;
    }
  }

  @Override
  public boolean isFinished() {
    // Finish once elevator is raised
    return elevatorRaised;
  }

  @Override
  public void end(boolean interrupted) {
    // Command is done
  }
}
