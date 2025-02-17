// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipperSubsystem;

/** Command to move the flipper into scoring position and release the coral. */
public class FlipperExtendAndScoreCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FlipperSubsystem m_subsystem;

  /**
   * Creates a new FlipperScoringCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlipperExtendAndScoreCommand(FlipperSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Claw is getting ready to score!");
  }

  @Override
  public void execute() {
    m_subsystem.extend();
    m_subsystem.letGo();
    m_subsystem.score();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Scoring of coral on branch is complete!");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
