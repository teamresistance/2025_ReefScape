package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipEleSubsystem;

/**
 * Command that checks for coral using banner sensors and initiates the gripping process. This
 * command can be used in two ways: 1. As a one-shot command triggered by a button press (isOneShot
 * = true) 2. As a continuous command that keeps checking for coral (isOneShot = false)
 */
public class FlipperGripperCmd extends Command {

  private final FlipEleSubsystem subsystem;
  private final boolean isOneShot;

  /**
   * Creates a FlipperGripperCmd that runs once when triggered.
   *
   * @param subsystem The subsystem to use
   */
  public FlipperGripperCmd(FlipEleSubsystem subsystem) {
    this(subsystem, true);
  }

  /**
   * Creates a FlipperGripperCmd with control over execution style.
   *
   * @param subsystem The subsystem to use
   * @param isOneShot If true, command finishes after one execution; if false, runs continuously
   */
  public FlipperGripperCmd(FlipEleSubsystem subsystem, boolean isOneShot) {
    this.subsystem = subsystem;
    this.isOneShot = isOneShot;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // flipperHoldingState now just processes banner sensor inputs and sets flags
    // which get handled in the periodic method with proper timing
    subsystem.flipperHoldingState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOneShot; // Only finish if configured as one-shot
  }
}
