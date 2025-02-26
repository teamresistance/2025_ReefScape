public class FlipperSubsystemUpdated {
    
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.Timer;

public class FlipperSubsystem extends SubsystemBase {
  private final Solenoid gripper = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  private final Solenoid flipper = new Solenoid(PneumaticsModuleType.CTREPCM, 9);
  private final Solenoid centeringSolenoid= new Solenoid(PneumaticsModuleType.CTREPCM, 7);
  private final DigitalInput coralDetector = new DigitalInput(0);
  private boolean hasCoral = false;
  private boolean isGripped = false;
  private boolean isInScoringPosition = false;
  private boolean isCentered= false;

  /** Creates a new FlipperSubsystem. */
  public FlipperSubsystem() {}

  public void grip() {
    hasCoral = coralDetector.get();
    gripper.set(hasCoral);
    isGripped = gripper.get();
  }

  public boolean updateHasCoral() {
    hasCoral = coralDetector.get();
    return hasCoral;
  }
  public void center(){
    centeringSolenoid.set(true);
    isCentered= centeringSolenoid.get();
  }

  public void letGo() {
    gripper.set(false);
    isGripped = false;
  }

  public void extend() {
    flipper.set(true);
    isInScoringPosition = true;
  }

  public void retract() {
    flipper.set(false);
    isInScoringPosition = false;
  }

  public void score() {
    Timer.delay(RobotConstants.kScoreTimeoutMilliseconds/1000);
    hasCoral = false;
  }

  @Override
  public void periodic() {
    //Scan for coral using DigitalInput.
    //If coral found, grip it after 200ms.
   if(isCentered==true){ 
    if (gripper.get() == false) {
      if (coralDetector.get() == true) {
          Timer.delay(RobotConstants.kGripperDelayMilliseconds/1000);
        gripper.set(true);
      }
    }
  }
    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Is Gripped?", isGripped);
    SmartDashboard.putBoolean("In Scoring Position?", isInScoringPosition);
  }

  @Override
  public void simulationPeriodic() {
    //Scan for coral using DigitalInput.
    //If coral found, grip it after 200ms.
    if(isCentered==true){
      if (gripper.get() == false) {
        if (coralDetector.get() == true) {
          
              Timer.delay(RobotConstants.kGripperDelayMilliseconds/1000);
              gripper.set(true);
      }
    }
    }

    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("Is Gripped?", isGripped);
    SmartDashboard.putBoolean("In Scoring Position?", isInScoringPosition);
  }
}

}
