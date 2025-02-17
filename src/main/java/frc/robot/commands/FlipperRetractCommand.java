// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FlipperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to move the flipper back to the receiving position. */
public class FlipperRetractCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final FlipperSubsystem m_subsystem;

  /**
   * Creates a new FlipperBackToReceivingPositionCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlipperRetractCommand(FlipperSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Flipper is getting ready to go back to receiving position!");
  }

  @Override
  public void execute() {
    m_subsystem.retract();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Ready to receive!");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}