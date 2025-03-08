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

  // Variables to control flipperHoldingState execution
  private int flipperCallCount = 0; // Counter for function calls
  private Timer flipperTimer = new Timer(); // Timer instance for non-blocking delay
  private boolean waitingForDelay = false; // Flag to track delay state
  private boolean isFlipperActive = false; // Flag to manage repeated calls

  public void flipperHoldingState() {
    if (flipperCallCount > 3) {
      // Reset everything if it exceeds the limit
      flipperCallCount = 0;
      waitingForDelay = false;
      isFlipperActive = false;
      return;
    }

    if (!(coralDetector1.get() && coralDetector2.get())) { // If one sees something
      if (!waitingForDelay) {
        gripper.set(false);

        coralCenterMechanism.setPulseDuration(0.5);
        coralCenterMechanism.startPulse();

        flipperTimer.reset();
        flipperTimer.start(); // Start non-blocking timer
        waitingForDelay = true;
        isFlipperActive = true; // Start tracking execution
      }
    } else {
      gripper.set(true);
      flipperCallCount = 0; // Reset count after gripping
      waitingForDelay = false;
      isFlipperActive = false;
    }
  }

  @Override
  public void periodic() {
    // Handle flipperHoldingState retries with a non-blocking delay
    if (isFlipperActive && waitingForDelay && flipperTimer.hasElapsed(0.7)) {
      flipperCallCount++; // Increment count after delay
      waitingForDelay = false; // Reset delay flag
      flipperHoldingState(); // Retry if needed
    }

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

  public boolean getHasCoral() {
    return believesHasCoral;
  }

  public boolean getIsntGripped() {
    return !gripper.get();
  }

  /** Flips the coral out. */
  public void flipperScore(double flipperDelay) {
    flipper.setPulseDuration(flipperDelay);
    flipper.startPulse();

    Timer.delay(0.75);
    gripper.set(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
