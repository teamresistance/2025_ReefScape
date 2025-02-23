package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  static Solenoid elevatorPusher1 =
      new Solenoid(Constants.SolenoidModuleType, Constants.kElevatorSolenoid1Channel);
  static Solenoid elevatorPusher2 =
      new Solenoid(Constants.SolenoidModuleType, Constants.kElevatorSolenoid2Channel);

  /**
   * The elevator subsystem is still available from both the codriver controls and the driver
   * controls. The codriver has the interface while the driver has buttons.
   */
  public ElevatorSubsystem() {}

  public static void raiseFirstStage() {
    elevatorPusher1.set(true);
  }

  public static void raiseSecondStage() {
    elevatorPusher2.set(true);
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
    Logger.recordOutput("Elevator/First Stage Up", elevatorPusher1.get());
    Logger.recordOutput("Elevator/Second Stage Up", elevatorPusher2.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
