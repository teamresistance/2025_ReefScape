package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

public class DynamicGoToTransform extends Command {

  private final Drive drive;
  private final Transform2d targetTransform;
  private final LoggedTunableNumber switchDistanceThreshold; // Changed to LoggedTunableNumber
  private Command currentCommand;

  public DynamicGoToTransform(
      Drive drive, Transform2d targetTransform, LoggedTunableNumber switchDistanceThreshold) {
    this.drive = drive;
    this.targetTransform = targetTransform;
    this.switchDistanceThreshold = switchDistanceThreshold;
    addRequirements(drive); // Declare that this command uses the drive subsystem
    currentCommand = null; // Initialize currentCommand
  }

  @Override
  public void initialize() {
    // Initial selection (can be omitted, as execute will handle it)
    updateCurrentCommand();
  }

  @Override
  public void execute() {
    updateCurrentCommand();
  }

  private void updateCurrentCommand() {
    boolean closeToTarget =
        drive.getPose().getTranslation().getDistance(targetTransform.getTranslation())
            < switchDistanceThreshold.get();

    if (closeToTarget) {
      if (currentCommand != DriveCommands.goToTransform(drive, targetTransform)) {
        if (currentCommand != null) {
          currentCommand.cancel();
        }

        currentCommand = DriveCommands.goToTransform(drive, targetTransform);
        currentCommand.initialize();
        System.out.println("Switched to goToTransform");
      }
    } else {
      if (currentCommand != DriveCommands.goToTransformWithPathFinder(drive, targetTransform)) {
        if (currentCommand != null) {
          currentCommand.cancel();
        }
        currentCommand = DriveCommands.goToTransformWithPathFinder(drive, targetTransform);
        currentCommand.initialize();
        System.out.println("Switched to goToTransformWithPathFinder");
      }
    }
    if (currentCommand != null) {
      currentCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (currentCommand != null) {
      currentCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Never finishes on its own; runs as long as the button is held
  }
}
