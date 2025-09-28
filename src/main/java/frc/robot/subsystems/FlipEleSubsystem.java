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
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.GRIPPER_SOLENOID_CHANNEL);
  public Solenoid centerer =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.CENTERER_SOLENOID_CHANNEL);
  private final Solenoid flipper =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.FLIPPER_SOLENOID_CHANNEL);
  private final DigitalInput coralDetector1 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL1);
  private final DigitalInput coralDetector2 = new DigitalInput(Constants.CORAL_SENSOR_CHANNEL2);

  // Variables to control flipperHoldingState execution
  private final int flipperCallCount = 0;
  private final Timer flipperTimer = new Timer();
  private boolean waitingForDelay = false;
  private boolean isFlipperActive = false;

  // State machine variables
  public boolean inHoldingState = false;
  private boolean inScoringMode = false;
  private final Timer centererTimer = new Timer();
  public boolean centererDelayActive = false;
  private final Timer elevatorTimer = new Timer();
  public boolean elevatorDelayActive = false;
  private final Timer elevatorLoweringTimer = new Timer();
  private boolean elevatorLoweringDelayActive = false;
  private final Timer resetButtonTimer = new Timer();
  private boolean resetButtonDelayActive = false;
  private final Timer flipperActuationTimer = new Timer();
  private boolean flipperActuationDelayActive = false;
  private final Timer gripperReleaseTimer = new Timer();
  private boolean gripperReleaseDelayActive = false;
  private final Timer gripperDelayTimer = new Timer();
  private boolean gripperDelayActive = false;
  public boolean centererClosePending = false;
  private boolean gripperClosePending = false;
  private boolean bannerSensorTriggered = false;

  // Default flipper-actuation delay
  private double flipperDelay = 2.0;
  // Configurable gripper-release delay (default 1.5s)
  private double gripperReleaseDelay = 1.5;

  public FlipEleSubsystem() {
    // Initialize centerer to be open
    centerer.set(false);
  }

  /** Raises lower stage of elevator without safety checks (use raiseFirstStageSafely instead) */
  public static void raiseFirstStage() {
    elevatorPusher1.set(true);
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

  public boolean hasObtainedCoral() {
    return coralDetector1.get() || coralDetector2.get();
  }

  /**
   * This method is called when the FlipperGripperCmd runs. It now just detects banner sensors and
   * sets flags for the periodic method to handle.
   */
  public void flipperHoldingState() {
    if (inScoringMode
        || elevatorPusher1.get()
        || elevatorPusher2.get()
        || elevatorDelayActive
        || elevatorLoweringDelayActive) {
      if (inHoldingState) {
        gripper.set(true);
      }
      centerer.set(false);
      return;
    }

    if (coralDetector1.get() || coralDetector2.get()) {
      bannerSensorTriggered = true;
      gripperClosePending = true;
      centererClosePending = true;

      if (!centererDelayActive) {
        centererTimer.reset();
        centererTimer.start();
        centererDelayActive = true;
        Logger.recordOutput("Flipper/Banner Sensor Delay Started", true);
        Logger.recordOutput("Flipper/Banner Sensor Delay Time", 0.5);
        SmartDashboard.putString("Banner State", "Banner sensor triggered, waiting 1.0s");
      }
    } else {
      gripper.set(false);
      bannerSensorTriggered = false;
      gripperClosePending = false;
      centererClosePending = false;

      if (centererDelayActive) {
        centererDelayActive = false;
        Logger.recordOutput("Flipper/Banner Sensor Delay Cancelled", true);
        SmartDashboard.putString("Banner State", "Banner sensor cleared - delay cancelled");
      }
    }
  }

  public void requestElevatorRaise() {
    if (elevatorPusher1.get() || elevatorDelayActive) {
      return;
    }

    centerer.set(false);
    inScoringMode = true;
    centererClosePending = false;
    bannerSensorTriggered = false;
    if (inHoldingState) {
      gripper.set(true);
    }

    elevatorTimer.reset();
    elevatorTimer.start();
    elevatorDelayActive = true;

    Logger.recordOutput("Elevator/Centerer Opened", true);
    Logger.recordOutput("Elevator/Waiting For Delay", true);
  }

  public void exitScoringMode() {
    inScoringMode = false;
  }

  public void raiseElevator(int level) {
    if (level <= 2) {
      lowerFirstStage();
      lowerSecondStage();
    } else if (level == 3) {
      if (inHoldingState) {
        requestElevatorRaise();
      } else {
        raiseFirstStage();
      }
      lowerSecondStage();
    } else if (level == 4) {
      if (inHoldingState) {
        requestElevatorRaise();
      } else {
        raiseFirstStage();
      }
      raiseSecondStage();
    }
  }

  public void toggleGripper() {
    gripper.set(!gripper.get());
  }

  public void resetHoldingState() {
    inHoldingState = false;
    bannerSensorTriggered = false;
    centererClosePending = false;
    gripperClosePending = false;
    centererDelayActive = false;
    gripperDelayActive = false;
    centerer.set(false);
    gripper.set(false);

    resetButtonTimer.reset();
    resetButtonTimer.start();
    resetButtonDelayActive = true;
    inScoringMode = true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Stage", elevatorPusher1.get());
    SmartDashboard.putBoolean("Second Stage", elevatorPusher2.get());
    Logger.recordOutput("Elevator/First Stage Up", elevatorPusher1.get());
    Logger.recordOutput("Elevator/Second Stage Up", elevatorPusher2.get());
    Logger.recordOutput("Flipper/Grip", gripper.get());
    Logger.recordOutput("Flipper/center", centerer.get());
    Logger.recordOutput("Flipper/flipper", flipper.get());

    if (!elevatorPusher1.get()
        && !elevatorPusher2.get()
        && inScoringMode
        && !elevatorLoweringDelayActive) {
      elevatorLoweringTimer.reset();
      elevatorLoweringTimer.start();
      elevatorLoweringDelayActive = true;
    }

    if (elevatorLoweringDelayActive && elevatorLoweringTimer.hasElapsed(2.0)) {
      inScoringMode = false;
      elevatorLoweringDelayActive = false;
    }

    if (elevatorPusher1.get() || elevatorPusher2.get()) {
      centerer.set(false);
      if (!inScoringMode) {
        inScoringMode = true;
      }
    }

    if (inScoringMode) {
      if (inHoldingState) {
        gripper.set(true);
      }
      centerer.set(false);
    }

    if (resetButtonDelayActive) {
      centerer.set(false);
      gripper.set(false);
      centererClosePending = false;
      gripperClosePending = false;
      if (resetButtonTimer.hasElapsed(0.0)) {
        inScoringMode = false;
        resetButtonDelayActive = false;
      }
    }

    if (flipperActuationDelayActive && flipperActuationTimer.hasElapsed(flipperDelay)) {
      inScoringMode = false;
      flipperActuationDelayActive = false;
      flipper.set(false);
    }

    // Process gripper release delay timer using configurable gripperReleaseDelay
    if (gripperReleaseDelayActive && gripperReleaseTimer.hasElapsed(gripperReleaseDelay)) {
      gripper.set(false);
      gripperReleaseDelayActive = false;
    }

    if (gripperDelayActive && gripperDelayTimer.hasElapsed(0.5)) {
      if (gripperClosePending) {
        gripper.set(true);
        gripperClosePending = false;
      }
      gripperDelayActive = false;
    }

    if (!inScoringMode
        && !elevatorPusher1.get()
        && !elevatorPusher2.get()
        && !elevatorDelayActive
        && !elevatorLoweringDelayActive
        && !resetButtonDelayActive
        && !flipperActuationDelayActive) {
      if (bannerSensorTriggered) {
        if (centererDelayActive && centererTimer.hasElapsed(0.3)) {
          if (centererClosePending) {
            centerer.set(true);
            centererClosePending = false;
            if (gripperClosePending && !gripperDelayActive) {
              gripperDelayTimer.reset();
              gripperDelayTimer.start();
              gripperDelayActive = true;
            }
          }
          inHoldingState = true;
          centererDelayActive = false;
        }
      }
    } else {
      bannerSensorTriggered = false;
    }

    if (elevatorDelayActive) {
      centerer.set(false);
      if (elevatorTimer.hasElapsed(0.0)) {
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

  /**
   * Flips the coral out. Keeps centerer open after scoring.
   *
   * @param flipperDelay seconds to hold the flipper out
   * @param gripperReleaseDelay seconds to wait before releasing the gripper
   */
  public void flipperScore(double flipperDelay, double gripperReleaseDelay) {
    this.gripperReleaseDelay = gripperReleaseDelay;
    CommandScheduler.getInstance().schedule(getFlipperCommand(flipperDelay, gripperReleaseDelay));
  }

  public Command getFlipperCommand(double flipperDelay_, double gripperReleaseDelay_) {
    return new InstantCommand(
        () -> {
          flipper.set(true);
          centerer.set(false);
          gripper.set(true);
          inScoringMode = true;
          inHoldingState = false;

          // configure delays
          this.flipperDelay = flipperDelay_;
          this.gripperReleaseDelay = gripperReleaseDelay_;

          // start timers
          gripperReleaseTimer.reset();
          gripperReleaseTimer.start();
          gripperReleaseDelayActive = true;

          flipperActuationTimer.reset();
          flipperActuationTimer.start();
          flipperActuationDelayActive = true;

          Logger.recordOutput("Flipper/Flipper Started", true);
          Logger.recordOutput("Flipper/Flipper Duration", flipperDelay_);
          Logger.recordOutput("Flipper/Gripper Release Delay", gripperReleaseDelay_);
          Logger.recordOutput("Flipper/Gripper Release Timer Started", true);
          SmartDashboard.putBoolean("Flipper Active", true);
          SmartDashboard.putBoolean("Gripper Release Delay Active", gripperReleaseDelayActive);
          SmartDashboard.putString(
              "Scoring Status",
              "Flipper out, gripper will release in " + gripperReleaseDelay_ + "s");
        });
  }

  public boolean isInHoldingState() {
    return inHoldingState;
  }

  public void setInScoringMode(boolean mode) {
    inScoringMode = mode;
  }

  public void closeGripper() {
    gripper.set(true);
  }

  public void openCenterer() {
    centerer.set(false);
  }

  public void resetAutoscoreState() {
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

    gripperReleaseTimer.stop();
    gripperReleaseTimer.reset();
    gripperReleaseTimer.start();
    gripperReleaseDelayActive = false;

    waitingForDelay = false;
    isFlipperActive = false;

    Logger.recordOutput("Flipper/Reset Autoscore State", true);
    Logger.recordOutput("Flipper/All Flipper Timers Reset", true);
  }

  @Override
  public void simulationPeriodic() {}
}
