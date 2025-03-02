package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PressureSubsystem extends SubsystemBase {

  private final Relay compressorRelay = new Relay(Constants.COMPRESSOR_RELAY_ID); // Relay port 0
  private static PneumaticsModuleType modType = PneumaticsModuleType.CTREPCM;
  private static int modID = Constants.PRESSURE_HUB_ID; // CAN adr, ID, of PDH
  private final AnalogInput pressureSensor = new AnalogInput(Constants.PRESSURE_SENSOR_ID);
  public static Compressor pcm = new Compressor(modID, modType);

  /** Subsystem for the compressor */
  public PressureSubsystem() {
    pcm.enableDigital();
  }

  private double getPressurePSI() {
    double voltage = pressureSensor.getVoltage();
    return (250 * (voltage / 5.0)) - 25; // Example conversion
  }

  @Override
  public void periodic() {
    if (getPressurePSI() < 90) {
      compressorRelay.set(Relay.Value.kForward); // Turn ON compressor
    } else if (getPressurePSI() > 120) {
      compressorRelay.set(Relay.Value.kOff); // Turn OFF compressor
    }
    // System.out.println("Compressor Enabled: " + pcm.isEnabled());
    // System.out.println("Pressure Switch Value: " + pcm.getPressureSwitchValue());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {}

}
