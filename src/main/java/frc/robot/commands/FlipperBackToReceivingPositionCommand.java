// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlipperSubsystem;

/** An example command that uses an example subsystem. */
public class FlipperBackToReceivingPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final FlipperSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FlipperBackToReceivingPositionCommand(FlipperSubsystem subsystem) {
    m_subsystem = subsystem; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Flipper is getting ready to go back to receiving position!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.flipperReadyToReceive();
  }  
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ready to receive!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
