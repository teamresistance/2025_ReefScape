// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.util.ArrayList;
import java.util.Optional;

public class InterfaceSubsystem extends SubsystemBase {
  public String currentBranchID;
  public String currentBranchLR = "M";
  public boolean lrButtonToggleState = true;
  public int currentBranchLevel;
  public SendableChooser<String> reefBranchChooser = new SendableChooser<String>();
  public SendableChooser<String> reefLevelChooser = new SendableChooser<String>();
  public Object[] reefBranchCombinations = new Object[3];

  /** Creates a new ExampleSubsystem. */
  public InterfaceSubsystem() {
  }

  public void storeBranchID(String branchID) {
    currentBranchID = branchID;
    System.out.println(currentBranchID);
  }

  public void storeBranchLevel(int branchLevel) {
    if (currentBranchLR.equals("M")) {
      currentBranchLevel = branchLevel;
    } else if (branchLevel == 2) {
      currentBranchLevel = 1;
    }
    System.out.println(currentBranchLevel);
  }

  public void toggleBranchLR() {
    lrButtonToggleState = !lrButtonToggleState;
    if (lrButtonToggleState == true) {
      currentBranchLR = "L";
    } else {
      currentBranchLR = "R";
    }
    System.out.println(currentBranchLR);
  }

  public void execute() {
    reefBranchCombinations = new Object[] {
        currentBranchID,
        currentBranchLR,
        currentBranchLevel };

    System.out.println("Branch Combination Stored:\n"
        + currentBranchID + "\n"
        + currentBranchLR + "\n"
        + currentBranchLevel + "\n");

  }

  public boolean branchCombinationExists() {
    return !Optional.ofNullable(reefBranchCombinations).isEmpty();
  }

  public Object[] getBranchCombination() {
    return reefBranchCombinations;
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Reef Branch Chooser", reefBranchChooser);
    SmartDashboard.putData("Reef Level Chooser", reefLevelChooser);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
