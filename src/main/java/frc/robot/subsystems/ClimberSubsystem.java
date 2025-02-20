package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  // This is never deactivated, only activated. Climber is one and done.
  private final Solenoid climberSV =
      new Solenoid(Constants.SolenoidModuleType, Constants.kClimberSolenoidChannel);

  /** Constructor for this subsystem. used to create an object in RobotContainer. */
  public ClimberSubsystem() {}

  /**
   * Sets the climberSV solenoid to the positon expressed by {@code state}. Also changes {@code
   * isActivated} to that state.
   *
   * @param state The intended state you want to set the climber solenoid.
   * @return void
   */
  public void activateClimber(boolean state) {
    climberSV.set(true);
    System.out.println("Climber is Activated: " + climberSV.get());
  }

  public boolean get() {
    return climberSV.get();
  }

  /*
   * This method is called every 20 ms.
   */
  @Override
  public void periodic() {
    // SmartDashboard reporting
    SmartDashboard.putBoolean("Climber is Activated:", climberSV.get());

    // Logging
    Logger.recordOutput("Climber/Activated", climberSV.get());
  }

  /*
   * This method is called every 20 ms in simulation.
   */
  public void simulationPeriodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber is Activated:", climberSV.get());
  }
}
