package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FlipEleSubsystem extends SubsystemBase {
  static Solenoid elevatorPusher1 =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.ELEVATOR_SOLENOID1_CHANNEL);
  static Solenoid elevatorPusher2 =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.ELEVATOR_SOLENOID2_CHANNEL);

  private static final boolean believesHasCoral = false;
  private static final Solenoid gripper =
      new Solenoid(
          2,
          Constants.SOLENOID_MODULE_TYPE,
          Constants
              .GRIPPER_SOLENOID_CHANNEL); // The pneumatics hub channels that we are using are 0, 2,
  public Solenoid centerer =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.CENTERER_SOLENOID_CHANNEL);
  // and 5
  private final Solenoid flipper =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.FLIPPER_SOLENOID_CHANNEL);
  private final DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL1);
  private final DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL2);

  // Variables to control flipperHoldingState execution
  private final int flipperCallCount = 0; // Counter for function calls
  private final Timer flipperTimer = new Timer(); // Timer instance for non-blocking delay
  private boolean waitingForDelay = false; // Flag to track delay state
  private boolean isFlipperActive = false; // Flag to manage repeated calls

  // State machine variables
  public boolean inHoldingState = false; // Tracks if we are in the coral holding state
  private boolean inScoringMode = false; // Tracks if we are in scoring mode (elevator up)
  private final Timer centererTimer = new Timer(); // Timer for centerer delays
  public boolean centererDelayActive = false; // Flag for tracking centerer delay
  private final Timer elevatorTimer = new Timer(); // Timer for elevator raising delay
  public boolean elevatorDelayActive = false; // Flag for tracking elevator delay
  private final Timer elevatorLoweringTimer = new Timer(); // Timer for post-elevator lowering delay
  private boolean elevatorLoweringDelayActive = false; // Flag for tracking post-lowering delay
  private final Timer resetButtonTimer = new Timer(); // Timer for post-reset button delay
  private boolean resetButtonDelayActive = false; // Flag for tracking post-reset button delay
  private final Timer flipperActuationTimer = new Timer(); // Timer for post-flipper actuation delay
  private boolean flipperActuationDelayActive = false; // Flag for tracking post-flipper delay
  private final Timer gripperReleaseTimer =
      new Timer(); // Timer for delayed gripper release after scoring
  private boolean gripperReleaseDelayActive = false; // Flag for tracking gripper release delay
  private final Timer gripperDelayTimer =
      new Timer(); // Timer for delayed gripper close after centerer
  private boolean gripperDelayActive = false; // Flag for tracking gripper delay
  public boolean centererClosePending = false; // Flag to indicate centerer needs to close
  private boolean gripperClosePending = false; // Flag to indicate gripper needs to close
  private boolean bannerSensorTriggered = false; // Flag to indicate a banner sensor was triggered
  private double flipperDelay = 2.0;

  public FlipEleSubsystem() {
    // Initialize centerer to be open
    centerer.set(false);
  }

  /** Raises lower stage of elevator without safety checks (use raiseFirstStageSafely instead) */
  public static void raiseFirstStage() {
    // Static method now just sets the solenoid directly
    // This is not ideal but preserved for backward compatibility
    // Should be managed through the instance method requestElevatorRaise() instead
    elevatorPusher1.set(true);
  }

  /** Raises upper stage of elevator */
  public static void raiseSecondStage() {
    elevatorPusher2.set(true);
  }

  /** Lowers lower stage of elevator */
  public static void lowerFirstStage() {
    elevatorPusher1.set(false);
    // Exit scoring mode and starting the post-lowering delay are handled in periodic
  }

  /** Lowers upper stage of elevator */
  public static void lowerSecondStage() {
    elevatorPusher2.set(false);
  }

  /**
   * Legacy method now redirects to requestElevatorRaise Use requestElevatorRaise() going forward
   */
  public void raiseFirstStageSafely() {
    // Redirect to the new method that handles proper sequencing
    requestElevatorRaise();
  }

  /**
   * Call this in periodic to handle the elevator raising sequence. This is necessary because the
   * elevator raising may span multiple cycles.
   */
  private void handleElevatorRaising() {}

  /**
   * This method is called when the FlipperGripperCmd runs. It now just detects banner sensors and
   * sets flags for the periodic method to handle.
   */
  public void flipperHoldingState() {
    // Skip banner sensor checks if elevator is up, going up, in scoring mode, or in post-lowering
    // delay
    if (inScoringMode
        || elevatorPusher1.get()
        || elevatorPusher2.get()
        || elevatorDelayActive
        || elevatorLoweringDelayActive) {
      // When in scoring mode or elevator is up/going up, ensure gripper is closed and centerer
      // stays open
      if (inHoldingState) {
        gripper.set(true);
      }
      centerer.set(false);
      return;
    }

    // Check if either banner sensor is tripped (returns false when triggered)
    if ((coralDetector1.get()
        || coralDetector2
            .get())) { // this should always be an or statement, should activate if either sensor is
      // tripped
      // A banner sensor has been tripped, set flags for periodic to handle
      bannerSensorTriggered = true;

      // Request gripper and centerer to close after delay
      gripperClosePending = true;
      centererClosePending = true;

      // Start the timer if not already running
      if (!centererDelayActive) {
        centererTimer.reset();
        centererTimer.start();
        centererDelayActive = true;
        Logger.recordOutput("Flipper/Banner Sensor Delay Started", true);
        Logger.recordOutput("Flipper/Banner Sensor Delay Time", 0.5);
        SmartDashboard.putString("Banner State", "Banner sensor triggered, waiting 1.0s");
      }
    } else {
      // No coral detected, open the gripper
      gripper.set(false);

      // Reset state flags
      bannerSensorTriggered = false;
      gripperClosePending = false;
      centererClosePending = false;

      // Important: Reset the timer and delay flag when banner sensors are clear
      if (centererDelayActive) {
        centererDelayActive = false;
        Logger.recordOutput("Flipper/Banner Sensor Delay Cancelled", true);
        SmartDashboard.putString("Banner State", "Banner sensor cleared - delay cancelled");
      }
    }
  }

  /**
   * Simple method to raise the elevator with an enforced delay between opening the centerer and
   * raising the elevator
   */
  public void requestElevatorRaise() {
    // CRITICAL: Make sure the elevator isn't already up or in the process of going up
    if (elevatorPusher1.get() || elevatorDelayActive) {
      return;
    }

    // First step: open centerer and prepare for elevator raising
    centerer.set(false); // OPEN the centerer (critical)

    // Enter scoring mode to block banner sensor checks
    inScoringMode = true;

    // Reset any pending centerer close commands
    centererClosePending = false;
    bannerSensorTriggered = false;

    // Ensure gripper stays closed if we were holding something
    if (inHoldingState) {
      gripper.set(true);
    }

    // Initialize delay timer if not already active
    elevatorTimer.reset();
    elevatorTimer.start();
    elevatorDelayActive = true;
    Logger.recordOutput("Elevator/Centerer Opened", true);
    Logger.recordOutput("Elevator/Waiting For Delay", true);

    // The actual elevator raising happens in periodic after the 1.5 second delay
  }

  /** Exit scoring mode when elevator is lowered */
  public void exitScoringMode() {
    inScoringMode = false;
  }

  /**
   * Method used in the elevator command. Raises elevator levels according to the level parameter.
   * Ensures centerer is open before raising elevator if in holding state.
   */
  public void raiseElevator(int level) {
    if (level <= 2) {
      lowerFirstStage();
      lowerSecondStage();
    } else if (level == 3) {
      if (inHoldingState) {
        raiseFirstStageSafely();
      } else {
        raiseFirstStage();
      }
      lowerSecondStage();
    } else if (level == 4) {
      if (inHoldingState) {
        raiseFirstStageSafely();
      } else {
        raiseFirstStage();
      }
      raiseSecondStage();
    }
  }

  /** Manually toggle gripper state (for driver button control) */
  public void toggleGripper() {
    gripper.set(!gripper.get());
  }

  /**
   * Manually reset holding state (can be used when cycling) Includes a safety delay before banner
   * sensors are monitored again. This forces both gripper and centerer to open regardless of sensor
   * input.
   */
  public void resetHoldingState() {
    // Reset all state flags
    inHoldingState = false;
    bannerSensorTriggered = false;

    // Cancel all active or pending operations
    centererClosePending = false;
    gripperClosePending = false;

    // Cancel any active timer-based operations
    centererDelayActive = false;
    gripperDelayActive = false;

    // IMPORTANT: Force both mechanisms to open position
    centerer.set(false); // Open centerer
    gripper.set(false); // Open gripper

    // Start a 2-second delay before banner sensors are monitored again
    resetButtonTimer.reset();
    resetButtonTimer.start();
    resetButtonDelayActive = true;

    // While the delay is active, we stay in "scoring mode" to prevent banner sensor processing
    inScoringMode = true;
  }

  @Override
  public void periodic() {
    // Logging / SmartDashboard info
    SmartDashboard.putBoolean("First Stage", elevatorPusher1.get());
    SmartDashboard.putBoolean("Second Stage", elevatorPusher2.get());
    Logger.recordOutput("Elevator/First Stage Up", elevatorPusher1.get());
    Logger.recordOutput("Elevator/Second Stage Up", elevatorPusher2.get());
    Logger.recordOutput("Flipper/Grip", gripper.get());
    Logger.recordOutput("Flipper/center", centerer.get());
    Logger.recordOutput("Flipper/flipper", flipper.get());

    // Check if elevator has been lowered, and start post-lowering delay
    if (!elevatorPusher1.get()
        && !elevatorPusher2.get()
        && inScoringMode
        && !elevatorLoweringDelayActive) {
      // Start the post-lowering delay timer (1 second)
      elevatorLoweringTimer.reset();
      elevatorLoweringTimer.start();
      elevatorLoweringDelayActive = true;
    }

    // Check if post-lowering delay has elapsed
    if (elevatorLoweringDelayActive
        && elevatorLoweringTimer.hasElapsed(
            2.0)) { // 2.0 seconds base delay after elevator comes down
      // Note: Additional delay for buttons CDGHKL is handled in InterfaceSubsystem
      // After delay, exit scoring mode to allow banner sensor processing
      inScoringMode = false;
      elevatorLoweringDelayActive = false;
    }

    // Check if elevator is up (either stage) and ensure centerer stays open
    if (elevatorPusher1.get() || elevatorPusher2.get()) {
      // When elevator is physically up, always keep centerer open
      centerer.set(false);
      // Also set scoring mode flag if it's not already set
      if (!inScoringMode) {
        inScoringMode = true;
      }
    }

    // Ensure gripper stays closed and centerer stays open when in scoring mode
    if (inScoringMode) {
      if (inHoldingState) {
        gripper.set(true);
      }
      // Always keep centerer open when in scoring mode
      centerer.set(false);
    }

    // ---- HANDLE CENTERING AND GRIPPING STATE MACHINE ----

    // Process reset button delay timer
    if (resetButtonDelayActive) {
      // While reset button delay is active, ensure mechanisms stay open regardless of other
      // conditions
      centerer.set(false);
      gripper.set(false);

      // Cancel any pending operations that might try to close the mechanisms
      centererClosePending = false;
      gripperClosePending = false;

      // Check if the delay has elapsed
      if (resetButtonTimer.hasElapsed(0.0)) { // 2 second delay after reset button
        // After delay, exit scoring mode to allow banner sensor processing
        inScoringMode = false;
        resetButtonDelayActive = false;
      }
    }

    // Process flipper actuation delay timer
    if (flipperActuationDelayActive
        && flipperActuationTimer.hasElapsed(
            flipperDelay)) { // 2 second delay after flipper actuation
      // After delay, exit scoring mode to allow banner sensor processing
      inScoringMode = false;
      flipperActuationDelayActive = false;
      flipper.set(false);
    }

    // Process gripper release delay timer - 1.0 seconds after flipper actuation
    if (gripperReleaseDelayActive
        && gripperReleaseTimer.hasElapsed(
            1.0)) { // Increased to 1.0 second delay before releasing gripper
      // After delay, open the gripper to release the game piece
      gripper.set(false);
      gripperReleaseDelayActive = false;
    }

    // Process gripper delay timer - 0.5 seconds after centerer is closed
    if (gripperDelayActive
        && gripperDelayTimer.hasElapsed(0.5)) { // 0.5 second delay before closing gripper
      // After delay, close the gripper to secure the game piece
      if (gripperClosePending) {
        gripper.set(true);
        gripperClosePending = false;
      }

      gripperDelayActive = false;
    }

    // Skip banner sensor handling while elevator is up, going up, in scoring mode, or in
    // any active delay period
    if (!inScoringMode
        && !elevatorPusher1.get()
        && !elevatorPusher2.get()
        && !elevatorDelayActive
        && !elevatorLoweringDelayActive
        && !resetButtonDelayActive
        && !flipperActuationDelayActive) {
      // Handle banner sensor being tripped only when elevator is completely down and not in
      // transition
      if (bannerSensorTriggered) {
        // If we're in the delay period after detecting coral
        if (centererDelayActive
            && centererTimer.hasElapsed(0.3)) { // Changed to 0.3 second delay before centering
          // The 1.0 second delay has elapsed

          // Close the centerer first if needed
          if (centererClosePending) {
            centerer.set(true);
            centererClosePending = false;

            // Start the gripper delay timer
            if (gripperClosePending && !gripperDelayActive) {
              gripperDelayTimer.reset();
              gripperDelayTimer.start();
              gripperDelayActive = true;
            }
          }

          // Set the holding state flag - gripper will close after its own delay
          inHoldingState = true;
          centererDelayActive = false;
        }
      }
    } else {
      // If elevator is up or going up, reset banner sensor flags
      bannerSensorTriggered = false;
    }

    // Handle elevator delay - CRITICAL for proper sequence
    if (elevatorDelayActive) {
      // Ensure centerer is ALWAYS open during delay
      centerer.set(false);

      // Check if delay has elapsed
      if (elevatorTimer.hasElapsed(0.0)) { // Changed to 0.75 second delay
        // After delay, raise the elevator
        elevatorPusher1.set(true);
        elevatorDelayActive = false;
      }
    }

    SmartDashboard.putBoolean("Gripper Closed", gripper.get());
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean("In Holding State", inHoldingState);
    SmartDashboard.putBoolean("In Scoring Mode", inScoringMode);
    SmartDashboard.putBoolean("Centerer Closed", !centerer.get());
    SmartDashboard.putBoolean("Elevator Delay Active", elevatorDelayActive);
    SmartDashboard.putBoolean("Post-Lowering Delay Active", elevatorLoweringDelayActive);
    SmartDashboard.putBoolean("Banner Sensor Triggered", bannerSensorTriggered);
    SmartDashboard.putBoolean("Centerer Delay Active", centererDelayActive);
    SmartDashboard.putBoolean("Gripper Delay Active", gripperDelayActive);
    SmartDashboard.putBoolean("Elevator Up", elevatorPusher1.get() || elevatorPusher2.get());
    SmartDashboard.putBoolean("Gripper Release Delay Active", gripperReleaseDelayActive);
    if (gripperReleaseDelayActive) {
      SmartDashboard.putNumber("Gripper Release Countdown", gripperReleaseTimer.get());
    }
    SmartDashboard.putBoolean(
        "Banner Sensors Disabled",
        inScoringMode
            || elevatorPusher1.get()
            || elevatorPusher2.get()
            || elevatorDelayActive
            || elevatorLoweringDelayActive
            || resetButtonDelayActive
            || flipperActuationDelayActive);
    SmartDashboard.putBoolean(
        "Coral Secured and Gripped",
        (gripper.get() && coralDetector1.get() && coralDetector2.get()));
  }

  /** Flips the coral out. Keeps centerer open after scoring. */
  public void flipperScore(double flipperDelay) {
    CommandScheduler.getInstance().schedule(getFlipperCommand(flipperDelay));
  }

  public Command getFlipperCommand(double flipperDelay_) {
    return new InstantCommand(
        () -> {
          // Start the flipper pulse - this keeps the flipper out for flipperDelay seconds
          //          flipper.setPulseDuration(flipperDelay);
          //          flipper.startPulse();
          flipper.set(true);

          // Ensure centerer is open during scoring
          centerer.set(false);

          // Keep gripper closed initially to hold game piece
          gripper.set(true);

          // Make sure we're in scoring mode
          inScoringMode = true;

          // Exit holding state
          inHoldingState = false;

          // IMMEDIATELY start the gripper release timer - don't wait
          gripperReleaseTimer.reset();
          gripperReleaseTimer.start();
          gripperReleaseDelayActive = true;

          // Start a 2-second delay before banner sensors are monitored again
          flipperActuationTimer.reset();
          flipperActuationTimer.start();
          flipperActuationDelayActive = true;
          flipperDelay = flipperDelay_;

          // Log the actions
          Logger.recordOutput("Flipper/Flipper Started", true);
          Logger.recordOutput("Flipper/Flipper Duration", flipperDelay);
          Logger.recordOutput("Flipper/Gripper Release Timer Started", true);
          Logger.recordOutput("Flipper/Gripper Release Delay Active", gripperReleaseDelayActive);
          SmartDashboard.putBoolean("Flipper Active", true);
          SmartDashboard.putBoolean("Gripper Release Delay Active", gripperReleaseDelayActive);
          SmartDashboard.putString("Scoring Status", "Flipper out, gripper will release in 1.0s");
        });
  }

  /** Get the current holding state */
  public boolean isInHoldingState() {
    return inHoldingState;
  }

  /** Set the scoring mode flag */
  public void setInScoringMode(boolean mode) {
    inScoringMode = mode;
  }

  /** Force gripper to closed position */
  public void closeGripper() {
    gripper.set(true);
  }

  /** Force centerer to open position */
  public void openCenterer() {
    centerer.set(false);
  }

  /**
   * Resets all flipper timing state after autoscore operation. This ensures consistent behavior
   * between successive scoring operations. Called when the right trigger is released.
   */
  public void resetAutoscoreState() {
    // Reset all timers to ensure clean state

    elevatorDelayActive = false;
    elevatorTimer.stop();
    elevatorTimer.reset();
    elevatorTimer.start();

    flipper.set(false);
    raiseElevator(0);
    flipperActuationTimer.stop();
    flipperActuationTimer.reset();
    flipperActuationTimer.start();
    flipperActuationDelayActive = false;

    // Make sure the gripper release timer is stopped
    gripperReleaseTimer.stop();
    gripperReleaseTimer.reset();
    gripperReleaseTimer.start();
    gripperReleaseDelayActive = false;

    // Don't immediately exit scoring mode, let natural cycle handle it
    // But clear any remaining flipper state flags
    waitingForDelay = false;
    isFlipperActive = false;

    // Log that we've reset all autoscore state
    Logger.recordOutput("Flipper/Reset Autoscore State", true);
    Logger.recordOutput("Flipper/All Flipper Timers Reset", true);
  }

  @Override
  public void simulationPeriodic() {}
}
