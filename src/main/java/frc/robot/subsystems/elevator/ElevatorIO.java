package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean elevatorPusher1;
    public boolean elevatorPusher2;
  }

  default void raiseFirstStage() {};

  default void raiseSecondStage() {};

  default void lowerFirstStage() {};

  default void lowerSecondStage() {};

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Updates the tunable numbers */
  default void updateTunableNumbers() {}
}
