package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.commands.DriveCommands.goToTransform;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AllianceTreePlace;
import frc.robot.FieldConstants.Place;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class InterfaceSubsystem extends SubsystemBase {

  private String pole = "a";
  private int level = 1;
  private boolean executing = false;

  private Transform2d targetTransform;
  private Transform2d leftRightOffset;

  private DriveSubsystem drive;
  private FlipperSubsystem flipper;
  private ElevatorSubsystem elevator;

  /**
   * Interface subsystem constructor - drive, flipper, elevator, subsystems are params from
   * robotcontainer. They are used because we cannot have two instances of any subsystem at the same
   * time.
   */
  public InterfaceSubsystem(
      DriveSubsystem drive, FlipperSubsystem flipper, ElevatorSubsystem elevator) {
    this.drive = drive;
    this.flipper = flipper;
    this.elevator = elevator;
  }

  /**
   * Drives to the location that is being pressed on the driver controller. Includes coral station,
   * climber, a specific reef pole
   *
   * <p>Example: Driver holding "A" button, robot auto-navigates to selected pole
   */
  private Transform2d getTranslationFromPlace(Place place) {

    AllianceTreePlace allianceplace = FieldConstants.getAllianceBranchFromBranch(place);
    return new Transform2d(
        new Translation2d(
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getX(),
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getY()),
        FieldConstants.getOffsetApriltagFromTree(allianceplace).getRotation());
  }

  public Command drive_command;

  /** Actually drives the robot to the position. Only called from driveToLoc() !!!! */
  private void executeDrive(Transform2d targetTransform, boolean isRight, boolean useOffset) {
    Logger.recordOutput("running work", true);
    if (useOffset) {
      if (isRight) {
        leftRightOffset = new Transform2d(0.50, -0.24, new Rotation2d(Units.degreesToRadians(0.0)));
      } else {
        leftRightOffset = new Transform2d(0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      }
    } else {
      leftRightOffset = new Transform2d(0.52, -0.05, new Rotation2d(0));
    }
    Logger.recordOutput("now putting things up", false);
    //    drive_command =
    //        DriveCommands.goToTransformWithPathFinderPlusOffset(drive, targetTransform,
    // leftRightOffset)
    //            .andThen(
    //                () -> {
    //                  Logger.recordOutput("now putting things up", true);
    //                  executeSelected();
    //                });

    drive_command =
        AutoBuilder.pathfindToPose(
                GeomUtil.transformToPose(targetTransform),
                new PathConstraints(
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.90, // TODO: CHANGE BACK
                    1.25, // TODO: CHANGE BACK
                    Units.degreesToRadians(440),
                    Units.degreesToRadians(720)),
                0.0 // Goal end velocity in meters/sec
                )
            .andThen(
                () -> {
                  elevator.raiseFromInterface(level);
                })
            .andThen(goToTransform(drive, targetTransform.plus(leftRightOffset)))
            .andThen(
                Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get())
                    .andThen(
                        () -> {
                          flipper.flipperScore(
                              useOffset
                                  ? Constants.SECONDS_TO_SCORE.get()
                                  : Constants.SECONDS_TO_SCORE.get() + 8);
                        })
                    //            .alongWith(DriveCommands.joystickDrive())
                    .andThen(Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() + 0.1))
                    .andThen(
                        () -> {
                          elevator.raiseFromInterface(0);
                        })
                    .andThen(Commands.waitSeconds(1.1))
                    .andThen(goToTransform(drive, targetTransform)));

    //    elevator.raiseFromInterface(level);
    CommandScheduler.getInstance().schedule(drive_command);
    Logger.recordOutput("offsetted pose", targetTransform.plus(leftRightOffset));
  }

  /**
   * Drives to the selected location Works by converting Pose2d of the branch selected to a
   * transform then pathfinder-ing to it.
   */
  public void driveToLoc(InterfaceExecuteMode loc) {
    boolean isRight = false;
    switch (loc) {
      case REEF:
        switch (pole) {
          case "a":
            targetTransform = getTranslationFromPlace(Place.A_TREE);
            break;
          case "b":
            targetTransform = getTranslationFromPlace(Place.B_TREE);
            isRight = true;
            break;
          case "c":
            targetTransform = getTranslationFromPlace(Place.C_TREE);
            break;
          case "d":
            targetTransform = getTranslationFromPlace(Place.D_TREE);
            isRight = true;
            break;
          case "e":
            targetTransform = getTranslationFromPlace(Place.E_TREE);
            break;
          case "f":
            targetTransform = getTranslationFromPlace(Place.F_TREE);
            isRight = true;
            break;
          case "g":
            targetTransform = getTranslationFromPlace(Place.G_TREE);
            break;
          case "h":
            targetTransform = getTranslationFromPlace(Place.H_TREE);
            isRight = true;
            break;
          case "i":
            targetTransform = getTranslationFromPlace(Place.I_TREE);
            break;
          case "j":
            targetTransform = getTranslationFromPlace(Place.J_TREE);
            isRight = true;
            break;
          case "k":
            targetTransform = getTranslationFromPlace(Place.K_TREE);
            break;
          case "l":
            targetTransform = getTranslationFromPlace(Place.L_TREE);
            isRight = true;
            break;
        }

        executeDrive(targetTransform, isRight, true);
        break;
      case ALGEE:
        switch (pole) {
          case "a":
            targetTransform = getTranslationFromPlace(Place.A_TREE);
            break;
          case "b":
            targetTransform = getTranslationFromPlace(Place.B_TREE);
            isRight = true;
            break;
          case "c":
            targetTransform = getTranslationFromPlace(Place.C_TREE);
            break;
          case "d":
            targetTransform = getTranslationFromPlace(Place.D_TREE);
            isRight = true;
            break;
          case "e":
            targetTransform = getTranslationFromPlace(Place.E_TREE);
            break;
          case "f":
            targetTransform = getTranslationFromPlace(Place.F_TREE);
            isRight = true;
            break;
          case "g":
            targetTransform = getTranslationFromPlace(Place.G_TREE);
            break;
          case "h":
            targetTransform = getTranslationFromPlace(Place.H_TREE);
            isRight = true;
            break;
          case "i":
            targetTransform = getTranslationFromPlace(Place.I_TREE);
            break;
          case "j":
            targetTransform = getTranslationFromPlace(Place.J_TREE);
            isRight = true;
            break;
          case "k":
            targetTransform = getTranslationFromPlace(Place.K_TREE);
            break;
          case "l":
            targetTransform = getTranslationFromPlace(Place.L_TREE);
            isRight = true;
            break;
        }
        executeDrive(targetTransform, isRight, false);
        break;
      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive(targetTransform, false, false);
        break;
      case DISABLE:
        //          forceStopExecution();
        drive_command.cancel();
        elevator.raiseFromInterface(0);
        break;
      case CLIMBER:
        targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
        executeDrive(targetTransform, false, false);
        break;
      case EXECUTE:
        if (!executing) {
          executeSelected();
        } else {
          forceStopExecution();
        }
        break;
      default: // Do nothing
    }
  }

  // ** Force-ends the execution and immediately retracts the elevator. */
  public void forceStopExecution() {
    executing = false;
    elevator.raiseFromInterface(0);
    // CommandScheduler.getInstance().cancel(drive_command);
    // drive_command.cancel();
  }

  /** Moves elevator to selected level and scores. */
  public void executeSelected() {
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR.get());
    flipper.flipperScore(Constants.SECONDS_TO_SCORE.get());
    Timer.delay(Constants.SECONDS_TO_SCORE.get() + 0.1);
    elevator.raiseFromInterface(0);
  }

  /**
   * Updates
   *
   * @param pole
   * @param level
   * @param updatepole
   * @param updatelevel
   */
  public void updateVars(String pole, int level, boolean updatepole, boolean updatelevel) {
    this.pole = updatepole ? pole : this.pole;
    this.level = updatelevel ? level : this.level;
  }

  @Override
  public void periodic() {
    // Logger / SmartDashboard info
    SmartDashboard.putString("Selected Pole", pole);
    SmartDashboard.putNumber("Selected Level", level);

    Logger.recordOutput("Selected Pole", pole);
    Logger.recordOutput("Selected Level", level);
  }
}
