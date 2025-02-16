// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlipperSubsystem extends SubsystemBase {

  private boolean believesHasCoral = false;
  private Solenoid gripper =
      new Solenoid(
          Constants.SolenoidModuleType,
          Constants
              .GripperSolenoidChannel); // The pneumatics hub channels that we are using are 0, 2,
  // and 5
  private Solenoid flipper =
      new Solenoid(Constants.SolenoidModuleType, Constants.FlipperSolenoidChannel);
  private Solenoid coralCenterMechanism =
      new Solenoid(Constants.SolenoidModuleType, Constants.CentererSolenoidChannel);
  private DigitalInput CoralDetector = new DigitalInput(0);

  /** Creates a new ExampleSubsystem. */
  public FlipperSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return the opposite of the value of said boolean state.
   */
  public void flipperHoldingState() {
    if (!believesHasCoral) {
      coralCenterMechanism.setPulseDuration(0.5);
      coralCenterMechanism.startPulse();
      while (coralCenterMechanism.get()) {}
      gripper.set(CoralDetector.get());
      believesHasCoral = true;
    } else if (believesHasCoral) {
      gripper.set(false);
      coralCenterMechanism.set(false);
    }
  }

  public void flipperScore() {
    flipper.setPulseDuration(1);
    flipper.startPulse();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Gripper Closed", gripper.get());
    SmartDashboard.putBoolean("Thinks has coral", believesHasCoral);
    SmartDashboard.putBoolean("Coral Secured and Gripped", (gripper.get() && CoralDetector.get()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
