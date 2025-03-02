package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FlipperSubsystem extends SubsystemBase {

  private static boolean believesHasCoral = false;
  private int recursions;
  private boolean detectorState1;
  private boolean detectorState2;
  private boolean gripperState;
  private boolean stopTryingGripper = false;
  private static Solenoid gripper =
      new Solenoid(
          Constants.SOLENOID_MODULE_TYPE,
          Constants
              .GRIPPER_SOLENOID_CHANNEL); // The pneumatics hub channels that we are using are 0, 2,
  // and 5
  private Solenoid flipper =
      new Solenoid(
          Constants.PRESSURE_HUB_ID,
          Constants.SOLENOID_MODULE_TYPE,
          Constants.FLIPPER_SOLENOID_CHANNEL);
  private Solenoid coralCenterMechanism =
      new Solenoid(
          Constants.PRESSURE_HUB_ID,
          Constants.SOLENOID_MODULE_TYPE,
          Constants.CENTERER_SOLENOID_CHANNEL);
  private DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL_1);
  private DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL_2);

  /** Subsystem handling coral intake and dropping onto branches/level1. */
  public FlipperSubsystem() {}

  /**
   * If the system thinks it has coral (dropped to the reef or had coral detected by both sensors),
   * it opens the gripper.
   *
   * <p>If the system doesn't think it has coral (after recieving from station), the coral centerer
   * will run every 2 seconds until both sensors detect a coral. While doing this, the system will
   * think that it has coral.
   *
   * <p>Can be cancelled by running the method again.
   */
  public void flipperHoldingState() {
    if (!(detectorState1 && detectorState2) && !stopTryingGripper) { // If one sees something
      if (!stopTryingGripper) {
        if (recursions >= 3) {
          stopTryingGripper = true;
        }
        gripper.set(false);

        coralCenterMechanism.setPulseDuration(0.5);
        coralCenterMechanism.startPulse();

        recursions++;
        Timer.delay(Constants.SECONDS_PER_CENTERING_ATTEMPT);

        // If it will stop trying to grip, do not continue, if it won't stop, ensure it won't stop
        if (!stopTryingGripper) {
          flipperHoldingState();
        } else {
          stopTryingGripper = false;
        }
      }
    } else {
      gripper.set(true);
      stopTryingGripper = false;
      recursions = 0;
    }
  }

  public boolean getIsGripped() {
    return gripperState;
  }

  public boolean getIsntGripped() {
    return !gripperState;
  }

  /** Flips the coral out. */
  public void flipperScore() {
    flipper.setPulseDuration(1);
    flipper.startPulse();
  }

  @Override
  public void periodic() {
    // Vars
    detectorState1 = coralDetector1.get();
    detectorState2 = coralDetector2.get();
    gripperState = gripper.get();
    // Automatic trigger if one detects a coral
    if (detectorState1 || detectorState2) {
      flipperHoldingState();
    }
    // Logging / smartdashboard
    Logger.recordOutput("Flipper/Gripper Is Closed", gripperState);
    Logger.recordOutput("Flipper/Robot Thinks Has Coral", believesHasCoral);
    Logger.recordOutput(
        "Flipper/Coral Secured", (gripperState && (detectorState1 && detectorState2)));
    SmartDashboard.putBoolean("Gripper Closed", gripperState);
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean(
        "Coral Secured and Gripped", (gripperState && (detectorState1 && detectorState2)));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
