package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipEleSubsystem;

/**
 * Command to toggle the gripper state manually (for driver control). Can be used to override the
 * automated gripper behaviors.
 */
public class GripperToggleCmd extends Command {

  private final FlipEleSubsystem subsystem;
  private final boolean resetHoldingState;

  /**
   * Creates a new GripperToggleCmd.
   *
   * @param subsystem The FlipEleSubsystem to control
   * @param resetHoldingState Whether to also reset the holding state
   */
  public GripperToggleCmd(FlipEleSubsystem subsystem, boolean resetHoldingState) {
    this.subsystem = subsystem;
    this.resetHoldingState = resetHoldingState;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.toggleGripper();

    // Optionally reset the holding state, which will also open the centerer
    if (resetHoldingState) {
      subsystem.resetHoldingState();
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
