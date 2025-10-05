package frc.robot.subsystems;

import static frc.robot.commands.DriveCommands.goToTransform;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AllianceTreePlace;
import frc.robot.FieldConstants.Place;
import frc.robot.commands.InterfaceActionCmd;
import frc.robot.commands.InterfaceActionCmd2;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for auto alignment to reef. Contains closest-face and left/right inversion in the
 * constructor
 */
class AutoAlignParams {

  Transform2d targetTransform;
  boolean isRight;

  AutoAlignParams(DriveSubsystem drive, boolean right) {

    Translation2d now = drive.getPose().getTranslation();

    // compensate for velocity (although x side is closer, it may be faster
    // to get to y side depending on the robot already moving that way, not
    // needing to change direction)
    now = now.plus(drive.getVelocity().getTranslation().times(0.15));

    Pose2d[] available = {
      FieldConstants.OFFSET_TAG_7, // Tag 7 --0
      FieldConstants.OFFSET_TAG_8, // Tag 8 --1
      FieldConstants.OFFSET_TAG_9, // Tag 9 Inverted right index 2
      FieldConstants.OFFSET_TAG_10, // Tag 10 Inverted right index 3
      FieldConstants.OFFSET_TAG_11, // Tag 11 Inverted right index 4
      FieldConstants.OFFSET_TAG_6, // Tag 6 --5
      FieldConstants.OFFSET_TAG_18, // Tag 18 --6
      FieldConstants.OFFSET_TAG_17, // Tag 17 --7
      FieldConstants.OFFSET_TAG_22, // Tag 22 Inverted right --8
      FieldConstants.OFFSET_TAG_21, // Tag 21 Inverted right --9
      FieldConstants.OFFSET_TAG_20, // Tag 20 Inverted right --10
      FieldConstants.OFFSET_TAG_19 // Tag 19 --11
    };

    Pose2d closestTag = available[0];
    double minDistance = now.getDistance(available[0].getTranslation());

    // Find the closest offset tag
    for (Pose2d tag : available) {
      double distance = now.getDistance(tag.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestTag = tag;
      }
    }

    // construct transform
    targetTransform =
        new Transform2d(
            new Translation2d(closestTag.getX(), closestTag.getY()), closestTag.getRotation());

    // check if the transform matches the far-side transforms
    // if so, invert the isRight value
    // keeps the left/right selection driver-relative instead of robot-relative
    isRight =
        (closestTag == available[2]
                || closestTag == available[3]
                || closestTag == available[4]
                || closestTag == available[8]
                || closestTag == available[9]
                || closestTag == available[10])
            != right;
  }
}

public class InterfaceSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;
  public final FlipEleSubsystem elevator;
  public Command drive_command = new InstantCommand();
  private String pole = "a";
  private int level = 1;
  private boolean executing = false;
  private Transform2d targetTransform = new Transform2d();
  private Transform2d leftRightOffset = new Transform2d();

  public InterfaceSubsystem(DriveSubsystem drive, FlipEleSubsystem elevator) {
    this.drive = drive;
    this.elevator = elevator;
  }

  public String getPole() {
    return pole;
  }

  public int getLevel() {
    return level;
  }

  public FlipEleSubsystem getElevator() {
    return elevator;
  }

  public Transform2d getTranslationFromPlace(Place place) {
    AllianceTreePlace allianceplace = FieldConstants.getAllianceBranchFromBranch(place);
    return new Transform2d(
        new Translation2d(
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getX(),
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getY()),
        FieldConstants.getOffsetApriltagFromTree(allianceplace).getRotation());
  }

  private void executeDrive(Transform2d targetTransform, boolean isRight, boolean useOffset) {
    if (useOffset) {
      if (isRight) {
        leftRightOffset = new Transform2d(0.50, -0.24, new Rotation2d(Units.degreesToRadians(0.0)));
      } else {
        leftRightOffset = new Transform2d(0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      }
    } else {
      leftRightOffset = new Transform2d(0.52, -0.05, new Rotation2d(0));
    }

    if (drive_command != null) {
      drive_command.cancel();
    }

    drive_command =
        (!drive.testingmode
                ? AutoBuilder.pathfindToPose(
                    GeomUtil.transformToPose(targetTransform), Constants.PATH_CONSTRAINTS, 0.0)
                : new InstantCommand(() -> {}))
            .andThen(
                () -> {
                  elevator.raiseElevator(level);
                })
            .andThen(goToTransform(drive, targetTransform.plus(leftRightOffset)))
            .andThen(Commands.runOnce(drive::stop))
            .andThen(
                Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get())
                    .andThen(
                        () -> {
                          // now call flipperScore with both delays
                          elevator.flipperScore(
                              useOffset
                                  ? Constants.SECONDS_TO_SCORE.get()
                                  : Constants.SECONDS_TO_SCORE.get() + 2,
                              getGripperReleaseDelayForLevel(),
                              this);
                        }))
            .andThen(
                useOffset
                    ? Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() + 0.3)
                    : Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 0.5))
            .andThen(
                () -> {
                  elevator.raiseElevator(0);
                })
            .andThen(Commands.waitSeconds(1.1))
            .andThen(goToTransform(drive, targetTransform))
            .andThen(
                () -> {
                  // no-op
                });

    CommandScheduler.getInstance().schedule(drive_command);
  }

  public void driveToLoc(
      InterfaceExecuteMode loc, InterfaceActionCmd stuff, int lvl, boolean isRight) {
    if (lvl != -1) level = lvl;
    AutoAlignParams params = new AutoAlignParams(drive, isRight);
    switch (loc) {
      case REEF:
        executeDrive(params.targetTransform, params.isRight, true);
        break;

      case ALGEE:
        executeDrive(params.targetTransform, params.isRight, false);
        break;

      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive(targetTransform, false, false);
        break;

      case DISABLE:
        drive_command.cancel();
        elevator.raiseElevator(0);
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

      default:
        // Do nothing
    }
  }

  private void executeDrive2(Transform2d targetTransform, boolean isRight, boolean useOffset) {
    if (useOffset) {
      if (isRight) {
        leftRightOffset = new Transform2d(0.52, -0.24, new Rotation2d(Units.degreesToRadians(0.0)));
      } else {
        leftRightOffset = new Transform2d(0.52, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      }
    } else {
      leftRightOffset = new Transform2d(0.52, -0.05, new Rotation2d(0));
    }

    boolean needsLongerDelay =
        "c".equals(pole)
            || "d".equals(pole)
            || "g".equals(pole)
            || "h".equals(pole)
            || "k".equals(pole)
            || "l".equals(pole);

    Logger.recordOutput("Scoring/Using Extended Delay", needsLongerDelay);
    SmartDashboard.putBoolean("Using Extended Delay", needsLongerDelay);

    if (drive_command != null) {
      drive_command.cancel();
    }

    drive_command =
        (!drive.testingmode
                ? AutoBuilder.pathfindToPose(
                    GeomUtil.transformToPose(targetTransform), Constants.PATH_CONSTRAINTS, 0.0)
                : new InstantCommand(() -> {}))
            .andThen(
                new InstantCommand(
                    () -> {
                      elevator.centererClosePending = false;
                      elevator.centerer.set(false);
                      elevator.setInScoringMode(true);
                    }))
            .andThen(Commands.waitSeconds(0.18))
            .andThen(
                () -> {
                  elevator.inHoldingState = true;
                  elevator.raiseElevator(level);
                })
            .andThen(
                () -> {
                  // updated flipperScore call
                  elevator.flipperScore(
                      Constants.SECONDS_TO_SCORE.get() + getExtraScoringTimeForLevel(),
                      getGripperReleaseDelayForLevel(),
                      this);
                })
            .andThen(goToTransform(drive, targetTransform.plus(leftRightOffset)))
            .andThen(Commands.runOnce(drive::stop))
            .andThen(
                Commands.waitSeconds(
                        Constants.SECONDS_TO_RAISE_ELEVATOR.get() + getElevatorRaiseWaitOffset())
                    .andThen(Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 1.3))
                    .andThen(
                        goToTransform(
                            drive,
                            targetTransform.plus(new Transform2d(0.2, -0.05, new Rotation2d(0)))))
                    .andThen(
                        goToTransform(
                            drive,
                            targetTransform.plus(new Transform2d(0.55, -0.05, new Rotation2d(0)))))
                    .andThen(Commands.runOnce(drive::stop))
                    .andThen(
                        () -> {
                          elevator.raiseElevator(0);
                        })
                    .andThen(
                        Commands.waitSeconds(
                            needsLongerDelay
                                ? Constants.SECONDS_TO_SCORE.get() - 0.9
                                : Constants.SECONDS_TO_SCORE.get() - 1.4))
                    .andThen(goToTransform(drive, targetTransform))
                    .andThen(
                        () -> {
                          // no-op
                        }));

    CommandScheduler.getInstance().schedule(drive_command);
  }

  public boolean safeToUngrip() {
    //    System.out.println(
    //        drive
    //                .getPose()
    //                .getTranslation()
    //                .getDistance(targetTransform.plus(leftRightOffset).getTranslation())
    //            + " is distance from target+offset");
    //    System.out.println(
    //        drive.getPose().getTranslation().getDistance(targetTransform.getTranslation())
    //            + " is distance from target with no offset");
    //    System.out.println(
    //        drive
    //            .getPose()
    //            .getTranslation()
    //            .getDistance(targetTransform.plus(leftRightOffset).getTranslation()));
    //    System.out.println(
    //        drive
    //                .getPose()
    //                .getTranslation()
    //                .getDistance(targetTransform.plus(leftRightOffset).getTranslation())
    //            < 0.05);
    return drive
            .getPose()
            .getTranslation()
            .getDistance(targetTransform.plus(leftRightOffset).getTranslation())
        < 0.07;
  }

  public void driveToLoc2(
      InterfaceExecuteMode loc,
      InterfaceActionCmd2 stuff,
      int lvl,
      boolean isRight,
      boolean singleDriver) {
    if (singleDriver) {
      if (lvl != -1) level = lvl;
      AutoAlignParams params = new AutoAlignParams(drive, isRight);
      targetTransform = params.targetTransform;
      switch (loc) {
        case REEF:
          executeDrive2(params.targetTransform, params.isRight, true);
          break;

        case ALGEE:
          executeDrive2(params.targetTransform, params.isRight, false);
          break;
        case CORAL:
          targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
          executeDrive2(targetTransform, false, false);
          break;
        case DISABLE:
          drive_command.cancel();
          elevator.raiseElevator(0);
          elevator.resetAutoscoreState();
          break;
        case CLIMBER:
          targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
          executeDrive2(targetTransform, false, false);
          break;
        case EXECUTE:
          if (!executing) {
            executeSelected();
          } else {
            forceStopExecution();
          }
          break;
        default:
          // Do nothing
      }
    } else {
      boolean isRighty = false;
      switch (loc) {
        case REEF:
          switch (pole) {
            case "a":
              targetTransform = getTranslationFromPlace(Place.A_TREE);
              break;
            case "b":
              targetTransform = getTranslationFromPlace(Place.B_TREE);
              isRighty = true;
              break;
            case "c":
              targetTransform = getTranslationFromPlace(Place.C_TREE);
              break;
            case "d":
              targetTransform = getTranslationFromPlace(Place.D_TREE);
              isRighty = true;
              break;
            case "e":
              targetTransform = getTranslationFromPlace(Place.E_TREE);
              break;
            case "f":
              targetTransform = getTranslationFromPlace(Place.F_TREE);
              isRighty = true;
              break;
            case "g":
              targetTransform = getTranslationFromPlace(Place.G_TREE);
              break;
            case "h":
              targetTransform = getTranslationFromPlace(Place.H_TREE);
              isRighty = true;
              break;
            case "i":
              targetTransform = getTranslationFromPlace(Place.I_TREE);
              break;
            case "j":
              targetTransform = getTranslationFromPlace(Place.J_TREE);
              isRighty = true;
              break;
            case "k":
              targetTransform = getTranslationFromPlace(Place.K_TREE);
              break;
            case "l":
              targetTransform = getTranslationFromPlace(Place.L_TREE);
              isRighty = true;
              break;
          }
          executeDrive2(targetTransform, isRighty, true);
          break;

        case ALGEE:
          switch (pole) {
            case "a":
              targetTransform = getTranslationFromPlace(Place.A_TREE);
              break;
            case "b":
              targetTransform = getTranslationFromPlace(Place.B_TREE);
              isRighty = true;
              break;
            case "c":
              targetTransform = getTranslationFromPlace(Place.C_TREE);
              break;
            case "d":
              targetTransform = getTranslationFromPlace(Place.D_TREE);
              isRighty = true;
              break;
            case "e":
              targetTransform = getTranslationFromPlace(Place.E_TREE);
              break;
            case "f":
              targetTransform = getTranslationFromPlace(Place.F_TREE);
              isRighty = true;
              break;
            case "g":
              targetTransform = getTranslationFromPlace(Place.G_TREE);
              break;
            case "h":
              targetTransform = getTranslationFromPlace(Place.H_TREE);
              isRighty = true;
              break;
            case "i":
              targetTransform = getTranslationFromPlace(Place.I_TREE);
              break;
            case "j":
              targetTransform = getTranslationFromPlace(Place.J_TREE);
              isRighty = true;
              break;
            case "k":
              targetTransform = getTranslationFromPlace(Place.K_TREE);
              break;
            case "l":
              targetTransform = getTranslationFromPlace(Place.L_TREE);
              isRighty = true;
              break;
          }
          executeDrive2(targetTransform, isRighty, false);
          break;
        case CORAL:
          targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
          executeDrive2(targetTransform, false, false);
          break;
        case DISABLE:
          drive_command.cancel();
          elevator.raiseElevator(0);
          elevator.resetAutoscoreState();
          break;
        case CLIMBER:
          targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
          executeDrive2(targetTransform, false, false);
          break;
        case EXECUTE:
          if (!executing) {
            executeSelected();
          } else {
            forceStopExecution();
          }
          break;
        default:
          // Do nothing
      }
    }
  }

  public void forceStopExecution() {
    executing = false;
    elevator.raiseElevator(0);
    CommandScheduler.getInstance().cancel(drive_command);
  }

  public void executeSelected() {
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR.get());
    elevator.flipperScore(Constants.SECONDS_TO_SCORE.get(), getGripperReleaseDelayForLevel(), this);
    Timer.delay(Constants.SECONDS_TO_SCORE.get() + 0.1);
    elevator.raiseElevator(0);
  }

  public void updateVars(String pole, int level, boolean updatepole, boolean updatelevel) {
    this.pole = updatepole ? pole : this.pole;
    this.level = updatelevel ? level : this.level;
  }

  private double getExtraScoringTimeForLevel() {
    return switch (level) {
      case 4 -> 5.5;
      case 3 -> 5.0;
      case 2 -> 4.0;
      default -> 0.0;
    };
  }

  private double getElevatorRaiseWaitOffset() {
    return switch (level) {
      case 4 -> 0.0;
      case 3 -> -0.5;
      case 2 -> -1.0;
      default -> 0.0;
    };
  }

  /** Returns the gripper release delay based on the current elevator level. */
  private double getGripperReleaseDelayForLevel() {
    return switch (level) {
      case 4 -> 1.5;
      case 3 -> 1.3;
      case 2 -> 1.2;
      default -> 1.5;
    };
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Selected Pole", pole);
    SmartDashboard.putNumber("Selected Level", level);
    Logger.recordOutput("Selected Pole", pole);
    Logger.recordOutput("Selected Level", level);
  }
}
