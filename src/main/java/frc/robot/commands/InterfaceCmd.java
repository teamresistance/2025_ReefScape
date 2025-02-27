// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InterfaceSubsystem2;

public class InterfaceCmd extends Command {

  private InterfaceSubsystem2 subsystem;
  private String pole;
  private int level;
  private boolean updatepole;
  private boolean updatelevel;

  public InterfaceCmd(
      InterfaceSubsystem2 subsystem,
      String pole,
      int level,
      boolean updatepole,
      boolean updatelevel) {
    addRequirements(subsystem);
    this.subsystem = subsystem;
    this.pole = pole;
    this.level = level;
    this.updatepole = updatepole;
    this.updatelevel = updatelevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.updateVars(pole, level, updatepole, updatelevel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
