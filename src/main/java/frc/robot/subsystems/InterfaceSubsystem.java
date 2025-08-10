package frc.robot.subsystems;

import static frc.robot.commands.DriveCommands.goToTransform;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private Transform2d targetTransform = new Transform2d();
  private Transform2d leftRightOffset = new Transform2d();
  private final Timer timer = new Timer();

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

  private void executeDrive(
      Transform2d targetTransform, boolean isRight, boolean useOffset, InterfaceActionCmd stuff) {
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
                              getGripperReleaseDelayForLevel());
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

      default:
        // Do nothing
    }
  }

  private SequentialCommandGroup rumbleCommand(CommandXboxController toRumble) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              toRumble.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
              toRumble.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            }),
        new WaitCommand(0.5),
        new InstantCommand(
            () -> {
              toRumble.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
              toRumble.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            }));
  }

  public void onTrue() {
    timer.stop();
    timer.reset();
  }

  private void startTimer() {
    timer.start();
  }

  private void executeDrive2(
      Transform2d targetTransform,
      boolean isRight,
      boolean useOffset,
      InterfaceActionCmd2 stuff,
      CommandXboxController toRumble) {

    if (useOffset) {
      leftRightOffset =
          isRight
              ? new Transform2d(0.52, -0.24, new Rotation2d())
              : new Transform2d(0.52, 0.11, new Rotation2d());
    } else {
      leftRightOffset = new Transform2d(0.52, -0.05, new Rotation2d());
    }

    boolean needsLongerDelay = "cdghkl".contains(pole);
    Logger.recordOutput("ExtendedDelay", needsLongerDelay);
    SmartDashboard.putBoolean("ExtendedDelay", needsLongerDelay);

    if (drive_command != null) {
      drive_command.cancel();
    }

    Pose2d checkPose = GeomUtil.transformToPose(targetTransform);
    Transform2d offsetTransform = targetTransform.plus(leftRightOffset);

    drive_command =
        new SequentialCommandGroup(
            drive.testingmode
                ? new InstantCommand(() -> {})
                : AutoBuilder.pathfindToPose(checkPose, Constants.PATH_CONSTRAINTS, 0.0),
            Commands.runOnce(drive::stop),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    Commands.runOnce(this::startTimer),
                    new ConditionalCommand(
                        goToTransform(drive, offsetTransform)
                            .andThen(drive::stop), // drive while timer less 2
                        rumbleCommand(toRumble), // rumble if timer more/equal 2
                        () -> timer.get() >= 2.0),
                    new SequentialCommandGroup(
                        new InstantCommand(
                            () -> {
                              elevator.centererClosePending = false;
                              elevator.centerer.set(false);
                              elevator.setInScoringMode(true);
                            }),
                        Commands.waitSeconds(0.18),
                        new InstantCommand(
                            () -> {
                              elevator.inHoldingState = true;
                              elevator.raiseElevator(level);
                            }),
                        new InstantCommand(
                            () -> {
                              elevator.flipperScore(
                                  Constants.SECONDS_TO_SCORE.get() + getExtraScoringTimeForLevel(),
                                  getGripperReleaseDelayForLevel());
                            }),
                        Commands.waitSeconds(
                            Constants.SECONDS_TO_RAISE_ELEVATOR.get()
                                + getElevatorRaiseWaitOffset()),
                        Commands.waitSeconds(Constants.SECONDS_TO_SCORE.get() - 1.3),
                        goToTransform(
                            drive,
                            targetTransform.plus(new Transform2d(0.2, -0.05, new Rotation2d(0)))),
                        goToTransform(
                            drive,
                            targetTransform.plus(new Transform2d(0.55, -0.05, new Rotation2d(0)))),
                        Commands.runOnce(drive::stop),
                        new InstantCommand(() -> elevator.raiseElevator(0)),
                        Commands.waitSeconds(
                            needsLongerDelay
                                ? Constants.SECONDS_TO_SCORE.get() - 0.9
                                : Constants.SECONDS_TO_SCORE.get() - 1.4),
                        goToTransform(drive, targetTransform))),
                new SequentialCommandGroup(
                    new InstantCommand(
                        () -> {
                          toRumble.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
                          toRumble.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
                        }),
                    new WaitCommand(0.5),
                    new InstantCommand(
                        () -> {
                          toRumble.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                          toRumble.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                        })),
                () ->
                    drive.getPose().getTranslation().getDistance(checkPose.getTranslation())
                        <= 0.9));

    CommandScheduler.getInstance().schedule(drive_command);
  }

  public void driveToLoc2(
      InterfaceExecuteMode loc, InterfaceActionCmd2 stuff, CommandXboxController toToRumble) {
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
        executeDrive2(targetTransform, isRight, true, stuff, toToRumble);
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
        executeDrive2(targetTransform, isRight, false, stuff, toToRumble);
        break;
      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive2(targetTransform, false, false, stuff, toToRumble);
        break;
      case DISABLE:
        drive_command.cancel();
        elevator.raiseElevator(0);
        elevator.resetAutoscoreState();
        break;
      case CLIMBER:
        targetTransform = getTranslationFromPlace(Place.MIDDLE_CAGE);
        executeDrive2(targetTransform, false, false, stuff, toToRumble);
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

  public void forceStopExecution() {
    executing = false;
    elevator.raiseElevator(0);
    CommandScheduler.getInstance().cancel(drive_command);
  }

  public void executeSelected() {
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR.get());
    elevator.flipperScore(Constants.SECONDS_TO_SCORE.get(), getGripperReleaseDelayForLevel());
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
