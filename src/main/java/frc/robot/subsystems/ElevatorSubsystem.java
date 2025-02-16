// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  static Solenoid elevatorPusher1 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid1Channel);
  static Solenoid elevatorPusher2 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid2Channel);

  /**
   * The elevator subsystem is still available from both the codriver controls and the driver
   * controls. The codriver has the interface while the driver has buttons.
   */
  public ElevatorSubsystem() {}

  public static void raiseFirstStage() {
    elevatorPusher1.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    elevatorPusher1.startPulse();
  }

  public static void raiseSecondStage() {
    elevatorPusher2.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    elevatorPusher2.startPulse();
  }

  public static void lowerFirstStage() {
    elevatorPusher1.set(false);
  }

  public static void lowerSecondStage() {
    elevatorPusher2.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Stage", elevatorPusher1.get());
    SmartDashboard.putBoolean("Second Stage", elevatorPusher2.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
