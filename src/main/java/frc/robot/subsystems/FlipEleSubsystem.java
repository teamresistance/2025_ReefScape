package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FlipEleSubsystem extends SubsystemBase {
  static Solenoid elevatorPusher1 =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.ELEVATOR_SOLENOID1_CHANNEL);
  static Solenoid elevatorPusher2 =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.ELEVATOR_SOLENOID2_CHANNEL);

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
  public Solenoid coralCenterMechanism =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.CENTERER_SOLENOID_CHANNEL);
  private DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL1);
  private DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL2);
  // Variables to control flipperHoldingState execution
  private int flipperCallCount = 0; // Counter for function calls
  private Timer flipperTimer = new Timer(); // Timer instance for non-blocking delay
  private boolean waitingForDelay = false; // Flag to track delay state
  private boolean isFlipperActive = false; // Flag to manage repeated calls

  public FlipEleSubsystem() {}

  /** Raises lower stage of elevator */
  public static void raiseFirstStage() {
    //    this.flipper.coralCenterMechanism.set(false);
    elevatorPusher1.set(true);
  }

  public void flipperHoldingState() {
    if (flipperCallCount > 1) {
      // Reset everything if it exceeds the limit
      flipperCallCount = 0;
      waitingForDelay = false;
      isFlipperActive = false;
      return;
    }

    if (!(coralDetector1.get() && coralDetector2.get())) { // If one sees something
      if (!waitingForDelay) {
        gripper.set(false);

        coralCenterMechanism.setPulseDuration(1.0);
        coralCenterMechanism.startPulse();

        flipperTimer.reset();
        flipperTimer.start(); // Start non-blocking timer
        waitingForDelay = true;
        isFlipperActive = true; // Start tracking execution
      }
    } else {
      gripper.set(true);
      //      coralCenterMechanism.set(true);
      flipperCallCount = 0; // Reset count after gripping
      waitingForDelay = false;
      isFlipperActive = false;
    }
  }

  /** Raises upper stage of elevator */
  public static void raiseSecondStage() {
    elevatorPusher2.set(true);
  }

  /** Lowers lower stage of elevator */
  public static void lowerFirstStage() {
    elevatorPusher1.set(false);
  }

  /** Lowers upper stage of elevator */
  public static void lowerSecondStage() {
    elevatorPusher2.set(false);
  }

  /**
   * Method used in the elevator command. Raises elevator levels according to the level parameter.
   */
  public void raiseFromInterface(int level) {
    //    if (level != 0) {
    //      this.flipper.coralCenterMechanism.set(false);
    //      Timer.delay(0.1);
    //    }

    if (level <= 2) {
      lowerFirstStage();
      lowerSecondStage();
    } else if (level == 3) {
      raiseFirstStage();
      lowerSecondStage();
    } else if (level == 4) {
      raiseFirstStage();
      raiseSecondStage();
    }
  }

  @Override
  public void periodic() {
    // Logging / SmartDashboard info
    SmartDashboard.putBoolean("First Stage", elevatorPusher1.get());
    SmartDashboard.putBoolean("Second Stage", elevatorPusher2.get());
    Logger.recordOutput("Elevator/First Stage Up", elevatorPusher1.get());
    Logger.recordOutput("Elevator/Second Stage Up", elevatorPusher2.get());

    // Handle flipperHoldingState retries with a non-blocking delay
    if (isFlipperActive && waitingForDelay && flipperTimer.hasElapsed(0.48)) {
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

  /** Flips the coral out. */
  public void flipperScore(double flipperDelay) {
    //    coralCenterMechanism.set(false);
    flipper.setPulseDuration(flipperDelay);
    flipper.startPulse();

    CommandScheduler.getInstance()
        .schedule(
            Commands.waitSeconds(0.75)
                .andThen(
                    () -> {
                      gripper.set(false);
                    }));
  }

  public Command getFlipperCommand(double flipperDelay) {
    return new InstantCommand(
            () -> {
              flipper.setPulseDuration(flipperDelay);
              flipper.startPulse();
            })
        .andThen(
            Commands.waitSeconds(0.75)
                .andThen(
                    () -> {
                      gripper.set(false);
                    }));
  }

  @Override
  public void simulationPeriodic() {}
}
