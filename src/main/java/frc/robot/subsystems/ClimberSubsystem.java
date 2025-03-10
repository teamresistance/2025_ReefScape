package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  // This is never deactivated after use. It exists to turn on.
  private final Solenoid climberSV =
      new Solenoid(2, Constants.SOLENOID_MODULE_TYPE, Constants.CLIMBER_SOLENOID_CHANNEL);

  public ClimberSubsystem() {}

  /** Activates the climber. Cannot be deactivated!! One-use. */
  public void activateClimber() {
    climberSV.set(true);
  }

  /** Returns if the climber has been activated or not */
  public boolean getClimberUsed() {
    return climberSV.get();
  }

  @Override
  public void periodic() {
    // Logging / SmartDashboard info
    SmartDashboard.putBoolean("Climber is Activated:", climberSV.get());
    Logger.recordOutput("Climber/Activated", climberSV.get());
  }

  public void simulationPeriodic() {}
}
