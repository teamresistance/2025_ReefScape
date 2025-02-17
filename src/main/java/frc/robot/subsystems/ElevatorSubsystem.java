// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

public class ElevatorSubsystem extends SubsystemBase {
  public Solenoid firstStageSolenoid =
      new Solenoid(
          HardwareConstants.pneumaticsModuleType,
          HardwareConstants.kSolenoid_firstStage_portNumber);

  public Solenoid secondStageSolenoid =
      new Solenoid(
          HardwareConstants.pneumaticsModuleType,
          HardwareConstants.kSolenoid_secondStage_portNumber);

  public boolean firstStageSolenoidUp = false;
  public boolean secondStageSolenoidUp = false;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return the opposite of the value of said boolean state.
   */
  public void raiseFirstStage() {
    firstStageSolenoid.setPulseDuration(HardwareConstants.kFirstStagePulseDurationSeconds);
    firstStageSolenoid.set(true);

    firstStageSolenoidUp = true;
  }

  public void raiseSecondStage() {
    secondStageSolenoid.setPulseDuration(HardwareConstants.kSecondStagePulseDurationSeconds);
    firstStageSolenoid.set(true);

    firstStageSolenoidUp = true;
  }

  public void lowerFirstStage() {
    firstStageSolenoid.set(false);

    firstStageSolenoidUp = false;
  }

  public void lowerSecondStage() {
    secondStageSolenoid.set(false);

    firstStageSolenoidUp = false;
  }

  public boolean getFirstStageSolenoidUp() {
    return firstStageSolenoidUp;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Stage Up?", firstStageSolenoidUp);
    SmartDashboard.putBoolean("Second Stage Up?", secondStageSolenoidUp);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
