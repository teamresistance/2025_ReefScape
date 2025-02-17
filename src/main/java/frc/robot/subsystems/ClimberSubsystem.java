package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

public class ClimberSubsystem extends SubsystemBase {

  // Hardware
  private final Solenoid climberSV = new Solenoid(
      HardwareConstants.pneumaticsModuleType, HardwareConstants.kClimberSolenoid_portNumber);

  // Variables

  public ClimberSubsystem() {
  }

  public void activateClimber(boolean state) {
    climberSV.setPulseDuration(1);
    climberSV.set(state);
    System.out.println("Climber is Activated: " + climberSV.get());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climber is Activated:", climberSV.get());
  }

  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber is Activated:", climberSV.get());
  }
}
