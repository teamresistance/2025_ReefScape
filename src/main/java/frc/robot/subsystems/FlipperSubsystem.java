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
          2,
          Constants.SOLENOID_MODULE_TYPE,
          Constants
              .GRIPPER_SOLENOID_CHANNEL); // The pneumatics hub channels that we are using are 0, 2,
  // and 5
  private Solenoid flipper =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.FLIPPER_SOLENOID_CHANNEL);
  private Solenoid coralCenterMechanism =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.CENTERER_SOLENOID_CHANNEL);
  private DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL1);
  private DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL2);

  /** Subsystem handling coral intake and dropping onto branches/level1. */
  public FlipperSubsystem() {}

  /**
   * If the flipper thinks it has/had coral (aka it dropped to the reef or tried gripping), it opens
   * the gripper. If the flipper doesn't think it has coral (right after recieving basically), it
   * centers then grips the coral.
   */
  public void flipperHoldingState() {
    if (!(coralDetector1.get() && coralDetector2.get())) { // If one sees something
      gripper.set(false);

      coralCenterMechanism.setPulseDuration(0.5);
      coralCenterMechanism.startPulse();

      Timer.delay(0.7);

      flipperHoldingState(); // TODO: ADD COUNTER FOR ONLY RECURSION 3 TIMES
    } else {
      gripper.set(true);
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

  public boolean getHasCoral() {
    return believesHasCoral;
  }

  public boolean getIsntGripped() {
    return !gripper.get();
  }

  /** Flips the coral out. */
  public void flipperScore() {
    flipper.setPulseDuration(5);
    flipper.startPulse();

    Timer.delay(0.75);
    gripper.set(false);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Flipper/Gripper Is Closed", gripper.get());
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
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
