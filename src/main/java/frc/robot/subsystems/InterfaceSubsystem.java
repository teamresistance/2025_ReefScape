package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    return new Transform2d(
        new Translation2d(
            FieldConstants.getSetPoint(place).getX(), FieldConstants.getSetPoint(place).getY()),
        FieldConstants.getSetPoint(place).getRotation());
  }

  /** Actually drives the robot to the position. Only called from driveToLoc() !!!! */
  private void executeDrive(Transform2d targetTransform, boolean isLeft, boolean useOffset) {
    if (useOffset) {
      if (isLeft){
        leftRightOffset = new Transform2d(0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      } else {
        leftRightOffset = new Transform2d(-0.50, 0.11, new Rotation2d(Units.degreesToRadians(0.0)));
      }
    } else {
      leftRightOffset = new Transform2d(0, 0, new Rotation2d(0));
    }
    DriveCommands.goToTransformWithPathFinderPlusOffset(
      drive,
      targetTransform,
      leftRightOffset);
  }

  /**
   * Drives to the selected location Works by converting Pose2d of the branch selected to a
   * transform then pathfinder-ing to it.
   */
  public void driveToLoc(InterfaceExecuteMode loc) {
    boolean isLeft = false;
    switch (loc) {
      case REEF:
        switch (pole) {
          case "a":
            targetTransform = getTranslationFromPlace(Place.A_TREE);
            break;
          case "b":
            targetTransform = getTranslationFromPlace(Place.B_TREE);
            isLeft = true;
            break;
          case "c":
            targetTransform = getTranslationFromPlace(Place.C_TREE);
            break;
          case "d":
            targetTransform = getTranslationFromPlace(Place.D_TREE);
            isLeft = true;
            break;
          case "e":
            targetTransform = getTranslationFromPlace(Place.E_TREE);
            break;
          case "f":
            targetTransform = getTranslationFromPlace(Place.F_TREE);
            isLeft = true;
            break;
          case "g":
            targetTransform = getTranslationFromPlace(Place.G_TREE);
            break;
          case "h":
            targetTransform = getTranslationFromPlace(Place.H_TREE);
            isLeft = true;
            break;
          case "i":
            targetTransform = getTranslationFromPlace(Place.I_TREE);
            break;
          case "j":
            targetTransform = getTranslationFromPlace(Place.J_TREE);
            isLeft = true;
            break;
          case "k":
            targetTransform = getTranslationFromPlace(Place.K_TREE);
            break;
          case "l":
            targetTransform = getTranslationFromPlace(Place.L_TREE);
            isLeft = true;
            break;
        }

        executeDrive(targetTransform, isLeft, true);
        break;
      case CORAL:
        targetTransform = getTranslationFromPlace(Place.LEFT_CORAL_STATION);
        executeDrive(targetTransform, false, false);
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
  }

  /** Moves elevator to selected level and scores. */
  public void executeSelected() {
    executing = true;
    elevator.raiseFromInterface(level);
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR);
    // flipper.flipperScore();
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
