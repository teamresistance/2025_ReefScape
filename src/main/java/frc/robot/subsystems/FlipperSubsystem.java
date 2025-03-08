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
  private int counter;
  private boolean detectorState1;
  private boolean detectorState2;
  private boolean gripperState;
  private boolean stopTryingGripper = false;
  private static Solenoid gripper =
      new Solenoid(
          2,
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
    if (!(coralDetector1.get() && coralDetector2.get())) { // If one sees something
      gripper.set(false);

      coralCenterMechanism.setPulseDuration(0.5);
      coralCenterMechanism.startPulse();
      counter++;
      Timer.delay(0.7);

      flipperHoldingState(); // TODO: ADD COUNTER FOR ONLY RECURSION 3 TIMES
    } else {
      gripper.set(true);
      counter = 0;
    }

    // if (coralDetector.get()) {
    //   gripper.set(true);
    // } else {
    //   gripper.set(false);
    //   coralCenterMechanism.setPulseDuration(0.5);
    //   coralCenterMechanism.startPulse();
    //   gripper.set(true);
    // }

    // if (!believesHasCoral) {

    //   while (coralCenterMechanism.get()) {}
    //   gripper.set(coralDetector.get());
    //   believesHasCoral = true;
    // } else if (believesHasCoral) {
    //   gripper.set(false);
    //   coralCenterMechanism.set(false);
    //   believesHasCoral = false;
    // }
  }

  public boolean getIsGripped() {
    return gripperState;
  }

  public boolean getIsntGripped() {
    return !gripperState;
  }

  /** Flips the coral out. */
  public void flipperScore(double flipperDelay) {
    flipper.setPulseDuration(flipperDelay);
    flipper.startPulse();

    Timer.delay(0.75);
    gripper.set(false);
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
        "Flipper/Coral Secured", (gripper.get() && coralDetector1.get() && coralDetector2.get()));
    SmartDashboard.putBoolean("Gripper Closed", gripper.get());
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean(
        "Coral Secured and Gripped",
        (gripper.get() && coralDetector1.get() && coralDetector2.get()));
  }

  @Override
  public void simulationPeriodic() {}
}
