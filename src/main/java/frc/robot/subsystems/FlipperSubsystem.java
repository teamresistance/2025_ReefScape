package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
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
  private DigitalInput coralDetector = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL);

  /** Subsystem handling coral intake and dropping onto branches/level1. */
  public FlipperSubsystem() {}

  /**
   * If the flipper thinks it has/had coral (aka it dropped to the reef or tried gripping), it opens
   * the gripper. If the flipper doesn't think it has coral (right after recieving basically), it
   * centers then grips the coral.
   */
  public void flipperHoldingState() {
    if (!believesHasCoral) {
      coralCenterMechanism.setPulseDuration(0.5);
      coralCenterMechanism.startPulse();
      while (coralCenterMechanism.get()) {}
      gripper.set(coralDetector.get());
      believesHasCoral = true;
    } else if (believesHasCoral) {
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
    Logger.recordOutput("Flipper/Coral Secured", (gripper.get() && coralDetector.get()));
    SmartDashboard.putBoolean("Gripper Closed", gripper.get());
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean("Coral Secured and Gripped", (gripper.get() && coralDetector.get()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
