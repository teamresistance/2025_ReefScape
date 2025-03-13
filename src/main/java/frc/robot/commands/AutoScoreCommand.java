package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class AutoScoreCommand extends Command {
  private final ClimberSubsystem climberSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final FlipperSubsystem flipperSubsystem;
  private final InterfaceSubsystem interfaceSubsystem;
  private final Vision vision;

  public AutoScoreCommand(
      ClimberSubsystem climberSubsystem,
      DriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      FlipperSubsystem flipperSubsystem,
      InterfaceSubsystem interfaceSubsystem,
      Vision vision) {
    this.climberSubsystem = climberSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.flipperSubsystem = flipperSubsystem;
    this.interfaceSubsystem = interfaceSubsystem;
    this.vision = vision;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(
        this.climberSubsystem,
        this.driveSubsystem,
        this.elevatorSubsystem,
        this.flipperSubsystem,
        this.interfaceSubsystem,
        this.vision);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {}

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {}

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is it is called when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
   * motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {}
}
