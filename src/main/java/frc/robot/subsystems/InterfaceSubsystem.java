// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class InterfaceSubsystem extends SubsystemBase {

  private String pole;
  private int level;

  private DriveSubsystem drive;
  private FlipperSubsystem flipper;
  private ElevatorSubsystem elevator;

  /**
   * Interface subsystem constructor. Drive, flipper, elevator, subsystems are params from
   * robotcontainer. They are used because we cannot have two instances of any subsystem at the same
   * time.
   */
  public InterfaceSubsystem(
      DriveSubsystem drive, FlipperSubsystem flipper, ElevatorSubsystem elevator) {
    this.drive = drive;
    this.flipper = flipper;
    this.elevator = elevator;
  }

  /**
   * Drives to the location that is being pressed on the driver controller. Includes coral station,
   * climber, a specific reef pole
   *
   * <p>Example: Driver holding "A" button, robot auto-navigates to selected pole
   */
  public void driveToLoc(String loc) {
    switch (loc) {
      case "REEF":
        switch (pole) {
          case "a":
            // Drive to pole A and align once nearby
            // Repeat this for remainder of positions once we have the transforms the robot will
            // need to be in
        }
      case "CORAL":
      // Drive commands to drive to coral station
      case "CLIMBER":
        // Drive commands to drive to climber
      case "EXECUTE":
        executeSelected();
    }
  }

  /** Moves elevator to selected level and scores. */
  public void executeSelected() {
    elevator.raiseFromInterface(level);
    Timer.delay(Constants.SECONDS_TO_RAISE_ELEVATOR);
    flipper.flipperScore();
    Timer.delay(Constants.SECONDS_TO_SCORE);
    elevator.raiseFromInterface(0);
  }

  /**
   * Updates
   *
   * @param pole
   * @param level
   * @param updatepole
   * @param updatelevel
   */
  public void updateVars(String pole, int level, boolean updatepole, boolean updatelevel) {
    this.pole = updatepole ? pole : this.pole;
    this.level = updatelevel ? level : this.level;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Selected Pole", pole);
    SmartDashboard.putNumber("Selected Level", level);
  }
}
