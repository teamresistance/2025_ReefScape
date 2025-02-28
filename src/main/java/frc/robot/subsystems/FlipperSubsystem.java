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
  private static Solenoid gripper =
      new Solenoid(
          Constants.SOLENOID_MODULE_TYPE,
          Constants
              .GRIPPER_SOLENOID_CHANNEL); // The pneumatics hub channels that we are using are 0, 2,
  // and 5
  private Solenoid flipper =
      new Solenoid(Constants.SOLENOID_MODULE_TYPE, Constants.FLIPPER_SOLENOID_CHANNEL);
  private Solenoid coralCenterMechanism =
      new Solenoid(Constants.SOLENOID_MODULE_TYPE, Constants.CENTERER_SOLENOID_CHANNEL);
  private DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_1_CHANNEL);
  private DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_2_CHANNEL);

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
    if (!believesHasCoral) {
      believesHasCoral = true;
      // Repeat until the coral is detected on both sensors or until the user cancels gripping
      while ((!coralDetector1.get() || !coralDetector2.get()) && believesHasCoral) {
        coralCenterMechanism.setPulseDuration(0.5);
        coralCenterMechanism.startPulse();
        Timer.delay(Constants.SECONDS_PER_CENTERING_ATTEMPT);
      }
      // If the loop ends normally (not cancelled), turn on gripper
      if (believesHasCoral) {
        gripper.set(true);
      }
    } else if (believesHasCoral) {
      // Release gripper and stop trying to center the coral
      gripper.set(false);
      coralCenterMechanism.set(false);
      believesHasCoral = false;
    }
  }

  public boolean getHasCoral() {
    return believesHasCoral;
  }

  public boolean getIsntGripped() {
    return !gripper.get();
  }

  /** Flips the coral out. */
  public void flipperScore() {
    flipper.setPulseDuration(1);
    flipper.startPulse();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Flipper/Gripper Is Closed", gripper.get());
    Logger.recordOutput("Flipper/Robot Thinks Has Coral", believesHasCoral);
    Logger.recordOutput(
        "Flipper/Coral Secured", (gripper.get() && (coralDetector1.get() && coralDetector2.get())));
    SmartDashboard.putBoolean("Gripper Closed", gripper.get());
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean(
        "Coral Secured and Gripped",
        (gripper.get() && (coralDetector1.get() && coralDetector2.get())));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
