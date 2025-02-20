package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class ElevatorIOSolenoid implements ElevatorIO {
  public Solenoid elevatorPusher1 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid1Channel);
  public Solenoid elevatorPusher2 =
      new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid2Channel);

  public ElevatorIOSolenoid() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorPusher1 = elevatorPusher1.get();
    inputs.elevatorPusher2 = elevatorPusher2.get();
  }

  public void raiseFirstStage() {
    elevatorPusher1.set(true);
  }

  public void raiseSecondStage() {
    elevatorPusher2.set(true);
  }

  public void lowerFirstStage() {
    elevatorPusher1.set(false);
  }

  public void lowerSecondStage() {
    elevatorPusher2.set(false);
  }

  @Override
  public void updateTunableNumbers() {
    // No tunable numbers
  }
}
