package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForTimeCmd extends Command {
  private final double waitTime;
  private final Timer timer = new Timer();

  public WaitForTimeCmd(double waitTime) {
    this.waitTime = waitTime;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(waitTime);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
