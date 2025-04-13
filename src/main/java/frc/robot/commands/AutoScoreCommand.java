package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.util.GeomUtil;

public class AutoScoreCommand extends SequentialCommandGroup {
  public AutoScoreCommand(
      InterfaceSubsystem reef, DriveSubsystem drive, FlipEleSubsystem elevator) {
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

    // Calculate left/right offset based on whether the target is on the right side
    Transform2d leftRightOffset =
        isRight
            ? new Transform2d(0.52, -0.24, new Rotation2d())
            : new Transform2d(0.52, 0.11, new Rotation2d());

    // Build the autoscore command chain
    addCommands(
        // 1. Pathfind to the target pose (converted from targetTransform)
        // Test without this initial target pose (because the one in pathplanner should in theory be the same)
        // AutoBuilder.pathfindToPose(
        //     GeomUtil.transformToPose(targetTransform), Constants.PATH_CONSTRAINTS, 0.0),
        new InstantCommand(() -> elevator.centerer.set(false)),
        Commands.waitSeconds(0.25),
        // 2. Raise the elevator to the selected level
        new InstantCommand(() -> elevator.raiseElevator(reef.getLevel())),

        // 3. Drive to a pose adjusted by the left/right offset
        DriveCommands.goToTransform(drive, targetTransform.plus(leftRightOffset)),
        Commands.runOnce(drive::stop),
        // 4. Wait for the elevator to raise (nonblocking wait)
        Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get()),

        // 5. Activate the flipper command for scoring (assume flipper.getFlipperCommand returns a
        // Command)
        elevator.getFlipperCommand(Constants.SECONDS_TO_SCORE.get() - 0.4), // was - 0.2

        // 6. Wait a short time after scoring
        Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 0.2), // was - 0.1 

        // 7. Lower the elevator back down
        new InstantCommand(() -> elevator.raiseElevator(0)));
  }
}
