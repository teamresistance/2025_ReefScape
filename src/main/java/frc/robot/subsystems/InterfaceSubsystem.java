package frc.robot.subsystems;

import static frc.robot.commands.DriveCommands.goToTransform;

import com.pathplanner.lib.auto.AutoBuilder;
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

public class InterfaceSubsystem extends SubsystemBase {

  private final DriveSubsystem drive;
  public final FlipEleSubsystem elevator;
  public Command drive_command = new InstantCommand();
  private String pole = "a";
  private int level = 1;
  private boolean executing = false;
  private Transform2d targetTransform;
  private Transform2d leftRightOffset;

  /**
   * Interface subsystem constructor - drive, flipper, elevator, subsystems are params from
   * robotcontainer. They are used because we cannot have two instances of any subsystem at the same
   * time.
   */
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

  /**
   * Get the elevator subsystem (used by commands to access elevator methods)
   *
   * @return The elevator subsystem
   */
  public FlipEleSubsystem getElevator() {
    return elevator;
  }

  /**
   * Drives to the location that is being pressed on the driver controller. Includes coral station,
   * climber, a specific reef pole
   *
   * <p>Example: Driver holding "A" button, robot auto-navigates to selected pole
   */
  public Transform2d getTranslationFromPlace(Place place) {

    AllianceTreePlace allianceplace = FieldConstants.getAllianceBranchFromBranch(place);
    return new Transform2d(
        new Translation2d(
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getX(),
            FieldConstants.getOffsetApriltagFromTree(allianceplace).getY()),
        FieldConstants.getOffsetApriltagFromTree(allianceplace).getRotation());
  }

  /** Actually drives the robot to the position. Only called from driveToLoc() !!!! */
  private void executeDrive(
      Transform2d targetTransform, boolean isRight, boolean useOffset, InterfaceActionCmd stuff) {

    //    Logger.recordOutput("running work", true);
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
        (drive.testingmode
                ? AutoBuilder.pathfindToPose(
                    GeomUtil.transformToPose(targetTransform),
                    Constants.PATH_CONSTRAINTS,
                    0.0 // Goal end velocity in meters/sec
                    )
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
                          elevator.flipperScore(
                              useOffset
                                  ? Constants.SECONDS_TO_SCORE.get()
                                  : Constants.SECONDS_TO_SCORE.get() + 8);
                        })
                    //            .alongWith(DriveCommands.joystickDrive())
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
                          //                          System.out.println("drive command executed and
                          // ended");
                          //
                          //                          stuff.finishparentCommand();
                          //                          stuff.end(true);
                          //                          drive.stop();
                        }));

    //    elevator.raiseFromInterface(level);
    CommandScheduler.getInstance().schedule(drive_command);
  }

  /**
   * Drives to the selected location Works by converting Pose2d of the branch selected to a
   * transform then pathfinder-ing to it.
   */
  public void driveToLoc(InterfaceExecuteMode loc, InterfaceActionCmd stuff) {
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

        executeDrive(targetTransform, isRight, true, stuff);
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
        executeDrive(targetTransform, isRight, false, stuff);
        break;
      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive(targetTransform, false, false, stuff);
        break;
      case DISABLE:
        //          forceStopExecution();
        drive_command.cancel();
        elevator.raiseElevator(0);
        break;
      case CLIMBER:
        targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
        executeDrive(targetTransform, false, false, stuff);
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

  /** Actually drives the robot to the position. Only called from driveToLoc() !!!! */
  private void executeDrive2(
      Transform2d targetTransform, boolean isRight, boolean useOffset, InterfaceActionCmd2 stuff) {

    //    Logger.recordOutput("running work", true);
    if (useOffset) {
      if (isRight) {
        leftRightOffset = new Transform2d(0.50, -0.24, new Rotation2d(Units.degreesToRadians(0.0)));
      } else {
        leftRightOffset = new Transform2d(0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      }
    } else {
      leftRightOffset = new Transform2d(0.52, -0.05, new Rotation2d(0));
    }

    // Determine if this is a CDGHKL button press which needs longer delay
    boolean needsLongerDelay =
        "c".equals(pole)
            || "d".equals(pole)
            || "g".equals(pole)
            || "h".equals(pole)
            || "k".equals(pole)
            || "l".equals(pole);

    // Log the delay decision
    Logger.recordOutput("Scoring/Using Extended Delay", needsLongerDelay);
    SmartDashboard.putBoolean("Using Extended Delay", needsLongerDelay);

    if (drive_command != null) {
      drive_command.cancel();
    }

    drive_command =
        (drive.testingmode
                ? AutoBuilder.pathfindToPose(
                    GeomUtil.transformToPose(targetTransform),
                    Constants.PATH_CONSTRAINTS,
                    0.0 // Goal end velocity in meters/sec
                    )
                : new InstantCommand(() -> {}))
            .andThen(
                new InstantCommand(
                    () -> {
                      elevator.centererClosePending = false;
                      elevator.centerer.set(false);
                    }))
            .andThen(Commands.waitSeconds(0.25))
            .andThen(
                () -> {
                  //                  // Use requestElevatorRaise for first stage when level is 3 or
                  // 4
                  //                  // This ensures proper delay between centerer opening and
                  // elevator rising
                  //                  if (level >= 3) {
                  //                    // Open centerer first, the elevator will raise after the
                  // delay in periodic
                  //                    elevator.requestElevatorRaise();
                  //
                  //                    // For level 4, also raise second stage after a small delay
                  //                    if (level == 4) {
                  //                      // Second stage will be raised in a separate command
                  //                      CommandScheduler.getInstance()
                  //                          .schedule(
                  //                              Commands.waitSeconds(0.8)
                  //                                  .andThen(() ->
                  // FlipEleSubsystem.raiseSecondStage()));
                  //                    }
                  //
                  //                    Logger.recordOutput("Interface/Using Safe Elevator Raise",
                  // true);
                  //                    SmartDashboard.putString("Elevator Action", "Using delayed
                  // elevator raise");
                  //                  } else {
                  //                    // For lower levels, just lower the elevator
                  elevator.inHoldingState = true;
                  elevator.raiseElevator(level);
                  //                  }
                })
            .andThen(goToTransform(drive, targetTransform.plus(leftRightOffset)))
            .andThen(Commands.runOnce(drive::stop))
            .andThen(
                Commands.waitSeconds(Constants.SECONDS_TO_RAISE_ELEVATOR.get())
                    .andThen(
                        () -> {
                          elevator.flipperScore(Constants.SECONDS_TO_SCORE.get());
                        })
                    //            .alongWith(DriveCommands.joystickDrive())
                    .andThen(Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() + 0.1))
                    .andThen(Commands.waitSeconds(1.1))
                    .andThen(
                        goToTransform(
                                drive,
                                targetTransform.plus(
                                    new Transform2d(0.50, -0.05, new Rotation2d(0))))
                            .andThen(Commands.runOnce(drive::stop))
                            .andThen(
                                () -> {
                                  elevator.flipperScore(Constants.SECONDS_TO_SCORE.get());
                                })
                            .andThen(
                                () -> {
                                  elevator.raiseElevator(0);
                                })
                            // Add conditional delay based on which button was pressed
                            .andThen(
                                Commands.waitSeconds(
                                    needsLongerDelay
                                        ? Constants.SECONDS_TO_SCORE.get() - 0.5
                                        : // 0.5s longer for CDGHKL
                                        Constants.SECONDS_TO_SCORE.get()
                                            - 1.0)) // Standard delay for others
                            .andThen(goToTransform(drive, targetTransform))
                            .andThen(
                                () -> {
                                  //                          System.out.println("drive command
                                  // executed and
                                  // ended");
                                  //
                                  //                          stuff.finishparentCommand();
                                  //                          stuff.end(true);
                                  //                          drive.stop();
                                })));

    //    elevator.raiseFromInterface(level);
    CommandScheduler.getInstance().schedule(drive_command);
  }

  /**
   * Drives to the selected location Works by converting Pose2d of the branch selected to a
   * transform then pathfinder-ing to it.
   */
  public void driveToLoc2(InterfaceExecuteMode loc, InterfaceActionCmd2 stuff) {
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

        executeDrive2(targetTransform, isRight, true, stuff);
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
        executeDrive2(targetTransform, isRight, false, stuff);
        break;
      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive2(targetTransform, false, false, stuff);
        break;
      case DISABLE:
        //          forceStopExecution();
        drive_command.cancel();
        elevator.raiseElevator(0);

        // Reset any state flags related to flipper timing when right trigger is released
        elevator.resetAutoscoreState();
        break;
      case CLIMBER:
        targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
        executeDrive2(targetTransform, false, false, stuff);
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
    elevator.raiseElevator(0);
    CommandScheduler.getInstance().cancel(drive_command);
    // drive_command.cancel();
  }

  /** Moves elevator to selected level and scores. */
  public void executeSelected() {
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR.get());
    elevator.flipperScore(Constants.SECONDS_TO_SCORE.get());
    Timer.delay(Constants.SECONDS_TO_SCORE.get() + 0.1);
    elevator.raiseElevator(0);
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
