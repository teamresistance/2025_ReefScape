package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ActivateClimberCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClimberSubsystem m_subsystem;

    public ActivateClimberCommand(ClimberSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    // runs when command gets initiatiated
    @Override
    public void initialize() {
        System.out.println("Activating Climber...");
    }

    // runs repeatedly until command is ended
    @Override
    public void execute() {
        m_subsystem.activateClimber(true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        System.out.println("Climber Activated!");
        return true;
    }

}