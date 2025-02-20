package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private boolean requestDown = false;
  private boolean requestLevel3 = false;
  private boolean requestLevel4 = false;


  /* System States */
  public enum ElevatorState {
    DOWN,
    LEVEL3,
    LEVEL4
  }

  private ElevatorState systemState = ElevatorState.DOWN;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updateTunableNumbers();
    Logger.processInputs("Elevator", inputs);

    Logger.recordOutput("Elevator/SystemState", systemState);

    if (requestDown) {
      systemState = ElevatorState.DOWN;
      io.lowerFirstStage();
      io.lowerSecondStage();
    } else if (requestLevel3) {
      systemState = ElevatorState.LEVEL3;
      io.raiseFirstStage();
      io.lowerSecondStage();
    } else if (requestLevel4) {
      systemState = ElevatorState.LEVEL4;
      io.raiseFirstStage();
      io.raiseSecondStage();
    }
  }


  public void requestIdle() {
    unsetAllRequests();
  }

  public void requestDown() {
    unsetAllRequests();
    requestDown = true;
  }

  public void requestLevel3() {
    unsetAllRequests();
    requestLevel3 = true;
  }

  public void requestLevel4() {
    unsetAllRequests();
    requestLevel4 = true;
  }

  private void unsetAllRequests() {
    requestDown = false;
    requestLevel3 = false;
    requestLevel4 = false;
  }


  public ElevatorState getSystemState() {
    return systemState;
  }
}
