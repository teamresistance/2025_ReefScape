package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DeferredCommand extends Command {
  private final Supplier<Command> commandSupplier;
  private Command command;

  public DeferredCommand(Supplier<Command> commandSupplier) {
    this.commandSupplier = commandSupplier;
  }

  @Override
  public void initialize() {
    // Build the command when this command is scheduled
    command = commandSupplier.get();
    if (command != null) {
      command.initialize();
    }
  }

  @Override
  public void execute() {
    if (command != null) {
      command.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (command != null) {
      command.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return command == null || command.isFinished();
  }
}
