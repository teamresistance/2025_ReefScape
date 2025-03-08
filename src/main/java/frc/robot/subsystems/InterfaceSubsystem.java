package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.InterfaceExecuteMode;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Place;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.DriveSubsystem;

public class InterfaceSubsystem extends SubsystemBase {

  private String pole = "";
  private int level = -1;
  private boolean executing = false;

  private Transform2d targetTransform;

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

  /**
   * Drives to the selected location Works by converting Pose2d of the branch selected to a
   * transform then pathfinder-ing to it.
   */
  public void driveToLoc(InterfaceExecuteMode loc) {
    switch (loc) {
      case REEF:
        switch (pole) {
          case "a":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.A_TREE).getX(),
                        FieldConstants.getSetPoint(Place.A_TREE).getY()),
                    FieldConstants.getSetPoint(Place.A_TREE).getRotation());
            break;
          case "b":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.B_TREE).getX(),
                        FieldConstants.getSetPoint(Place.B_TREE).getY()),
                    FieldConstants.getSetPoint(Place.B_TREE).getRotation());
            break;
          case "c":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.C_TREE).getX(),
                        FieldConstants.getSetPoint(Place.C_TREE).getY()),
                    FieldConstants.getSetPoint(Place.C_TREE).getRotation());
            break;
          case "d":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.D_TREE).getX(),
                        FieldConstants.getSetPoint(Place.D_TREE).getY()),
                    FieldConstants.getSetPoint(Place.D_TREE).getRotation());
            break;
          case "e":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.E_TREE).getX(),
                        FieldConstants.getSetPoint(Place.E_TREE).getY()),
                    FieldConstants.getSetPoint(Place.E_TREE).getRotation());
            break;
          case "f":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.F_TREE).getX(),
                        FieldConstants.getSetPoint(Place.F_TREE).getY()),
                    FieldConstants.getSetPoint(Place.F_TREE).getRotation());
            break;
          case "g":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.G_TREE).getX(),
                        FieldConstants.getSetPoint(Place.G_TREE).getY()),
                    FieldConstants.getSetPoint(Place.G_TREE).getRotation());
            break;
          case "h":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.H_TREE).getX(),
                        FieldConstants.getSetPoint(Place.H_TREE).getY()),
                    FieldConstants.getSetPoint(Place.H_TREE).getRotation());
            break;
          case "i":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.I_TREE).getX(),
                        FieldConstants.getSetPoint(Place.I_TREE).getY()),
                    FieldConstants.getSetPoint(Place.I_TREE).getRotation());
            break;
          case "j":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.J_TREE).getX(),
                        FieldConstants.getSetPoint(Place.J_TREE).getY()),
                    FieldConstants.getSetPoint(Place.J_TREE).getRotation());
            break;
          case "k":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.K_TREE).getX(),
                        FieldConstants.getSetPoint(Place.K_TREE).getY()),
                    FieldConstants.getSetPoint(Place.K_TREE).getRotation());
            break;
          case "l":
            targetTransform =
                new Transform2d(
                    new Translation2d(
                        FieldConstants.getSetPoint(Place.L_TREE).getX(),
                        FieldConstants.getSetPoint(Place.L_TREE).getY()),
                    FieldConstants.getSetPoint(Place.L_TREE).getRotation());
            break;
        }
        DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
            .andThen(DriveCommands.goToTransform(drive, targetTransform))
            .beforeStarting(
                () -> {
                  DriveCommands.goToTransform(drive, targetTransform).cancel();
                  DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                });
        break;
      case CORAL:
        targetTransform =
            new Transform2d(
                new Translation2d(
                    FieldConstants.getSetPoint(Place.LEFT_CORAL_STATION).getX(),
                    FieldConstants.getSetPoint(Place.LEFT_CORAL_STATION).getY()),
                FieldConstants.getSetPoint(Place.LEFT_CORAL_STATION).getRotation());
        DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
            .andThen(DriveCommands.goToTransform(drive, targetTransform))
            .beforeStarting(
                () -> {
                  DriveCommands.goToTransform(drive, targetTransform).cancel();
                  DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                });
        break;
      case CLIMBER:
        targetTransform =
            new Transform2d(
                new Translation2d(
                    FieldConstants.getSetPoint(Place.MIDDLE_CAGE).getX(),
                    FieldConstants.getSetPoint(Place.MIDDLE_CAGE).getY()),
                FieldConstants.getSetPoint(Place.MIDDLE_CAGE).getRotation());

        DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
            .andThen(DriveCommands.goToTransform(drive, targetTransform))
            .beforeStarting(
                () -> {
                  DriveCommands.goToTransform(drive, targetTransform).cancel();
                  DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                });
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
  }

  /** Moves elevator to selected level and scores. */
  public void executeSelected() {
    executing = true;
    elevator.raiseFromInterface(level);
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR);
    flipper.flipperScore();
    Timer.delay(Constants.SECONDS_TO_SCORE);
    elevator.raiseFromInterface(0);
    executing = false;
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
  }
}
