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
  public boolean isInScoringPosition = false;
  public boolean isInReceivingPosition = false;
  public boolean isGripped = false;
  public boolean hasCoral = false;

  Solenoid gripper =
      new Solenoid(
          Constants.SolenoidModuleType,
          Constants
              .GripperSolenoidChannel); // The pneumatics hub channels that we are using are 0, 2,
  // and 5
  Solenoid flipper = new Solenoid(Constants.SolenoidModuleType, Constants.FlipperSolenoidChannel);
  Solenoid coralCenterMechanism =
      new Solenoid(Constants.SolenoidModuleType, Constants.CentererSolenoidChannel);
  DigitalInput CoralDetector = new DigitalInput(0);

  /** Creates a new ExampleSubsystem. */
  public FlipperSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return the opposite of the value of said boolean state.
   */
  public void grip() {
    coralCenterMechanism.setPulseDuration(0.5);
    coralCenterMechanism.startPulse();
    hasCoral = CoralDetector.get();
    gripper.set(hasCoral);
    isGripped = gripper.get();
  }

  public void letGo() {
    gripper.set(false);
  }

  /// public void clawSpinner(){
  // double motorTurnAmount= angleToEncoderTicks(90);
  /// rotator.set(ControlMode.position, motorTurnAmount);
  // }
  public void flipperReadyToScore() {
    flipper.setPulseDuration(1);
    flipper.startPulse();
    isInScoringPosition = true;
  }

  public void flipperReadyToReceive() {
    flipper.set(false);
    isInReceivingPosition = true;
  }

  /// public void clawUnspinner(){
  /// rotator.set(ControlMode.Position, -motorTurnAmount);
  /// }

  public void score() {
    hasCoral = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Is Gripped?", isGripped);
    SmartDashboard.putBoolean("In Scoring Position?", isInScoringPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
