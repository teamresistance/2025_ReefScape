// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class InferfaceSubsystem extends SubsystemBase {
  /** Subsystem that handles the codriver interface inputs. */
  public InferfaceSubsystem() {}

  private int level;
  private int pos;
  private boolean rl = false; // Left is false, right is true

  /**
   * ChooseVars(level, pos, rl) runs when an interface button is pressed but it does NOT execute the
   * code
   *
   * <p>- level is an int from 0-3 representing the targeted reef level. level 1 and 0 may have the
   * same elevator height BUT level 0 drops the coral into the trough and level 1 puts it on the
   * reef!!!! level 0 also works for recieving coral (maybe...)
   *
   * <p>- pos is the int position around the reef from 0-5 - rl is an int either 0 or 1, 0
   * representing left and 1 representing right
   *
   * <p>- If you do not want to change a value, set it to -1 in the method (ie if i only wanted to
   * execute the code (by setting exec to true))
   *
   * <p>- All my other values would be -1. example: ChooseVars(-1, 4, -1) would set the position to
   * 4 and change no other values in the subsystem. if you put new ChooseVars(0, 4, 0), it would set
   * the level and rl variables to 0!!!
   *
   * @param level
   * @param pos
   * @param rl (0 = left, 1 = right)
   */
  public void chooseVars(int level, int pos, int rl) {
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

  /**
   * This is a FULL sequence of moving to position, going to left/right on that position, raising
   * the elevator, scoring, then lowering the elevator.
   *
   * @param drive (the drive subsystem)
   * @param elevator (the elevator subsystem)
   * @param flipper (the flipper subsystem)
   * @param level
   * @param pos
   * @param rl (0 = left, 1 = right)
   */
  @SuppressWarnings(
      "static-access") // VSC tries warning me that it should be accessed statically, it's 100% fine
  // currently.
  public void chooseReef(
      Drive drive,
      ElevatorSubsystem elevator,
      FlipperSubsystem flipper,
      int level,
      int pos,
      int rl) {
    // Drive robot to the position chosen
    switch (pos) {
      case 0:
        Transform2d targetTransform =
            new Transform2d(
                new Translation2d(14.35, 4.31), new Rotation2d(Units.degreesToRadians(-178.0)));
        DriveCommands.goToTransformWithPathFinder(drive, targetTransform)
            .andThen(DriveCommands.goToTransform(drive, targetTransform))
            .beforeStarting(
                () -> {
                  DriveCommands.goToTransform(drive, targetTransform).cancel();
                  DriveCommands.goToTransformWithPathFinder(drive, targetTransform).cancel();
                });
        break;
        // Repeat this for the rest of the positions with the proper locations
    }
    Timer.delay(Constants.kTimeToGetToReefSeconds);
    // Raise elevator to level inputted
    switch (level) {
      case 0:
        elevator.lowerFirstStage();
        elevator.lowerSecondStage();
        break;
      case 1:
        elevator.lowerFirstStage();
        elevator.lowerSecondStage();
        break;
      case 2:
        elevator.raiseFirstStage();
        elevator.lowerSecondStage();
        Timer.delay(Constants.kTimeToRaiseStageSeconds);
        break;
      case 3:
        elevator.raiseFirstStage();
        elevator.raiseSecondStage();
        Timer.delay(Constants.kTimeToRaiseStageSeconds * 2);
        break;
    }
    // Shift to right/left based on rl variable
    switch (rl) {
      case 0:
        // Move the robot to the left a tiny bit
        break;
      case 1:
        // Move the robot to the right a tiny bit
        break;
    }
    Timer.delay(Constants.kTimeToShiftSeconds);
    // Score
    flipper.flipperScore();
    Timer.delay(Constants.kTimeToScoreSeconds);
    // Lower elevator when done
    elevator.lowerFirstStage();
    elevator.lowerSecondStage();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Level", level);
    SmartDashboard.putNumber("Position", pos);
    SmartDashboard.putBoolean("Side (right=true)", rl);
    // This method will be called once per scheduler run
  }
}
