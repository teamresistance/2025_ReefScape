// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhysicalReefInterfaceSubsystem extends SubsystemBase {
  /** Creates a new PhysicalReefSubsystem. */
  public PhysicalReefInterfaceSubsystem() {}

  private int level;
  private int pos;
  private boolean rl = false; // Left is false, right is true

  public void ChooseReef() {
    /*
    ChooseReef()
    - Runs whenever the exec variable is true in a ChooseReefCmd().
    - This method makes the robot actually do what the saved variables are. If you had pressed elevator level 4, position 2, toggled
     to left all before running this method the robot would use those saved values to change the elevator height to 4 and move to left pos 2
    - This also resets the values when running, so if you for whatever reason wanted to go to the same place you'd have to
     press every button again
    */
    SmartDashboard.putString(
        "Last Selection", "Level = " + level + ", Position = " + pos + ", L/R = " + rl);
    level = 0;
    pos = 0;
    rl = false;
  }

  // Add code to make the robot go to X location

  /*
   ChooseVars(level, pos, rl) runs when an interface button is pressed but it does NOT execute the code
   - level is an int from 0-3 representing the targeted reef level. level 1 and 0 may have the same elevator height BUT level 0 drops
   the coral into the trough and level 1 puts it on the reef!!!! level 0 also works for recieving coral (maybe...)
   - pos is the int position around the reef from 0-5
   - rl is an int either 0 or 1, 0 representing left and 1 representing right
   
   - if you do not want to change a value, set it to -1 in the method (ie if i only wanted to execute the code (by setting exec to true))
   - all my other values would be -1. example: ChooseVars(-1, 4, -1) would set the position to 4 and change no other
   values in the subsystem. if you put new ChooseVars(0, 4, 0), it would set the level and rl variables to 0!!!
   */
  public void ChooseVars(int level, int pos, int rl) {
    if (rl == 1) {
      this.rl = !this.rl;
    }
    if (level > -1) {
      this.level = level;
    }
    if (pos > -1) {
      this.pos = pos;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Level", level);
    SmartDashboard.putNumber("Position", pos);
    SmartDashboard.putBoolean("Side (right=true)", rl);
    // This method will be called once per scheduler run
  }
}
