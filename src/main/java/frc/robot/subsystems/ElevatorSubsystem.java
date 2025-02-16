// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  static Solenoid ElevatorPusher1 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid1Channel);
  static Solenoid ElevatorPusher2 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid2Channel);

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {}

  public static void raiseFirstStage() {
    ElevatorPusher1.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    ElevatorPusher1.startPulse();
  }

  public static void raiseSecondStage() {
    ElevatorPusher2.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    ElevatorPusher2.startPulse();
  }

  public static void lowerFirstStage() {
    ElevatorPusher1.set(false);
  }

  public static void lowerSecondStage() {
    ElevatorPusher2.set(false);
    lowerFirstStage();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Stage", ElevatorPusher1.get());
    SmartDashboard.putBoolean("Second Stage", ElevatorPusher2.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
