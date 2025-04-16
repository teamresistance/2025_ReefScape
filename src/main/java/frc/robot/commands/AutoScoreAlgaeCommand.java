package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.FlipEleSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoScoreAlgaeCommand extends SequentialCommandGroup {

  private boolean needsLongerDelay;

  public AutoScoreAlgaeCommand(
      InterfaceSubsystem reef, DriveSubsystem drive, FlipEleSubsystem elevator, boolean doCoral) {
    // Determine target transform based on selected pole
    String pole = reef.getPole();
    boolean isRight = "b d f h j l".contains(pole);
    Transform2d targetTransform;
    switch (pole) {
      case "a":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.A_TREE);
        break;
      case "b":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.B_TREE);
        break;
      case "c":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.C_TREE);
        break;
      case "d":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.D_TREE);
        break;
      case "e":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.E_TREE);
        break;
      case "f":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.F_TREE);
        break;
      case "g":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.G_TREE);
        break;
      case "h":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.H_TREE);
        break;
      case "i":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.I_TREE);
        break;
      case "j":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.J_TREE);
        break;
      case "k":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.K_TREE);
        break;
      case "l":
        targetTransform = reef.getTranslationFromPlace(FieldConstants.Place.L_TREE);
        break;
      default:
        throw new IllegalStateException("Unexpected pole: " + pole);
    }

    // Determine if this is a CDGHKL button press which needs longer delay
    needsLongerDelay =
        "c".equals(pole)
            || "d".equals(pole)
            || "g".equals(pole)
            || "h".equals(pole)
            || "k".equals(pole)
            || "l".equals(pole);

    // Calculate left/right offset based on whether the target is on the right side
    Transform2d leftRightOffset =
        isRight
            ? new Transform2d(0.52, -0.24, new Rotation2d())
            : new Transform2d(0.52, 0.11, new Rotation2d());

    Transform2d middleOffset = new Transform2d(0.52, -0.05, new Rotation2d(0));
    //
    //    // Build the autoscore command chain
    //    addCommands(
    //        Commands.waitSeconds(0.2),
    //        // 2. Raise the elevator to the selected level
    //        new InstantCommand(() -> elevator.raiseElevator(reef.getLevel())),
    //
    //        // 3. Drive to a pose adjusted by the left/right offset
    //        DriveCommands.goToTransform(drive, targetTransform.plus(middleOffset)),
    //        Commands.runOnce(drive::stop),
    //        // 4. Wait for the elevator to raise (nonblocking wait)
    //        Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get()),
    //
    //        // 5. Activate the flipper command for scoring (assume flipper.getFlipperCommand
    // returns a
    //        // Command)
    //        elevator.getFlipperCommand(Constants.SECONDS_TO_SCORE.get() - 0.4), // was - 0.2
    //
    //        // 6. Wait a short time after scoring
    //        Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 0.2), // was - 0.1
    //
    //        // 7. Lower the elevator back down
    //        new InstantCommand(() -> elevator.raiseElevator(0)));

    if (doCoral) {
      addCommands(
          new InstantCommand(() -> elevator.centerer.set(false)),
          Commands.waitSeconds(0.18), // was 0.25
          new InstantCommand(
              () -> {
                elevator.inHoldingState = true;
                elevator.raiseElevator(reef.getLevel());
              }),
          new InstantCommand(
              () -> {
                elevator.flipperScore(Constants.SECONDS_TO_SCORE.get() + 5.5);
              }),
          DriveCommands.goToTransform(drive, targetTransform.plus(leftRightOffset)),
          Commands.runOnce(drive::stop),
          Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get()),
          Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 1.0),
          DriveCommands.goToTransform(
              drive, targetTransform.plus(new Transform2d(0.2, -0.05, new Rotation2d(0)))),
          DriveCommands.goToTransform(
              drive, targetTransform.plus(new Transform2d(0.55, -0.05, new Rotation2d(0)))),
          Commands.runOnce(drive::stop),
          new InstantCommand(() -> elevator.raiseElevator(0)),
          Commands.waitSeconds(
              needsLongerDelay
                  ? Constants.SECONDS_TO_SCORE.get() - 0.9
                  : Constants.SECONDS_TO_SCORE.get() - 1.4),
          DriveCommands.goToTransform(drive, targetTransform));
    } else {
      addCommands(
          new InstantCommand(() -> elevator.centerer.set(false)),
          Commands.waitSeconds(0.2),
          new InstantCommand(() -> elevator.raiseElevator(reef.getLevel())),
          new InstantCommand(
              () -> {
                elevator.flipperScore(Constants.SECONDS_TO_SCORE.get() + 5.5);
              }),
          DriveCommands.goToTransform(drive, targetTransform.plus(middleOffset)),
          Commands.runOnce(drive::stop),
          Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get()),
          elevator.getFlipperCommand(Constants.SECONDS_TO_SCORE.get() - 0.4),
          new InstantCommand(() -> elevator.raiseElevator(0)),
          Commands.waitSeconds(
              needsLongerDelay
                  ? Constants.SECONDS_TO_SCORE.get() - 0.9
                  : Constants.SECONDS_TO_SCORE.get() - 1.4),
          DriveCommands.goToTransform(drive, targetTransform));
    }
  }
}
