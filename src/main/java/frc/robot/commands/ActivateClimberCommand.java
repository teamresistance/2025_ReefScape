package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ActivateClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem subsystem;

  public ActivateClimberCommand(ClimberSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // runs when command gets initiatiated
  @Override
  public void initialize() {}

  // runs repeatedly until command is ended
  @Override
  public void execute() {
    subsystem.activateClimber();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
