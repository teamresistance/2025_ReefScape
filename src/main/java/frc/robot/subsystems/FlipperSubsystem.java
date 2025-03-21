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

  public FlipperSubsystem() {}

  /**
   * Activates the centerer, then checks if the banner sensors detect coral.
   *
   * <p>If not: Tries again, maximum of 3 reattempts before it gives up
   *
   * <p>If coral detected: Grip the coral and reset recursions counter
   */
  public void flipperHoldingState() {
    if (!(detectorState1 && detectorState2) && !stopTryingGripper) { // If one sees something
      if (!stopTryingGripper) {
        // Check for recursions, if >3 stop
        if (recursions >= 3) {
          stopTryingGripper = true;
        }
        gripper.set(false);

        // Center croal
        coralCenterMechanism.setPulseDuration(Constants.SECONDS_OF_CENTERING_PULSE);
        coralCenterMechanism.startPulse();

        recursions++;
        Timer.delay(Constants.SECONDS_PER_CENTERING_ATTEMPT);

        // If it will stop trying to grip, do not continue, if it won't stop, ensure it won't stop
        if (!stopTryingGripper) {
          flipperHoldingState();
        } else {
          stopTryingGripper = false;
          recursions = 0;
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
    // Variable updates
    detectorState1 = coralDetector1.get();
    detectorState2 = coralDetector2.get();
    gripperState = gripper.get();

    // Automatic trigger if one detects a coral
    if (detectorState1 || detectorState2) {
      flipperHoldingState();
    }

    // Logging / SmartDashboard
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
  public void simulationPeriodic() {}
}
