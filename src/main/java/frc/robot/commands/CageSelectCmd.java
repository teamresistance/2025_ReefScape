package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class CageSelectCmd extends Command {

  public static final Pose2d INNER_CAGE_RED =
      new Pose2d(9.786 - 0.15, 3.02 - 0.05, Rotation2d.fromDegrees(90.0));
  public static final Pose2d MIDDLE_CAGE_RED =
      new Pose2d(9.786 - 0.15, 1.894 - 0.05, Rotation2d.fromDegrees(90.0));
  public static final Pose2d OUTER_CAGE_RED =
      new Pose2d(9.786 - 0.15, 0.766, Rotation2d.fromDegrees(90.0));
  public static final Pose2d INNER_CAGE_BLUE =
      new Pose2d(7.736 + 0.15, 5.072 + 0.05, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d MIDDLE_CAGE_BLUE =
      new Pose2d(7.736 + 0.15, 6.170 + 0.05, Rotation2d.fromDegrees(-90.0));
  public static final Pose2d OUTER_CAGE_BLUE =
      new Pose2d(7.736 + 0.15, 7.254, Rotation2d.fromDegrees(-90.0));
  public static Optional<DriverStation.Alliance> ally;
  private static int currentCageId = 0;
  private final int cageId;

  public CageSelectCmd(int cageId) {
    if (cageId < 0 || cageId > 2) {
      throw new IllegalArgumentException("Invalid cage ID: " + cageId);
    }
    this.cageId = cageId;
    currentCageId = cageId;
  }

  @Override
  public void initialize() {
    ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      DriverStation.Alliance team = ally.get();
      Pose2d targetPose;
      switch (cageId) {
        case 0 ->
            targetPose = (team == DriverStation.Alliance.Red) ? OUTER_CAGE_RED : OUTER_CAGE_BLUE;
        case 1 ->
            targetPose = (team == DriverStation.Alliance.Red) ? MIDDLE_CAGE_RED : MIDDLE_CAGE_BLUE;
        case 2 ->
            targetPose = (team == DriverStation.Alliance.Red) ? INNER_CAGE_RED : INNER_CAGE_BLUE;
        default -> throw new IllegalArgumentException("Invalid cage ID: " + cageId);
      }
      RobotContainer.setCageClimb(targetPose);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  public static class CycleCageCmd extends Command {
    @Override
    public void initialize() {
      currentCageId = (currentCageId + 1) % 3; // Cycle through 0, 1, 2
      Logger.recordOutput("Climber/CurrentCage", currentCageId);
      new CageSelectCmd(currentCageId).schedule();
    }

    @Override
    public boolean isFinished() {
      return true;
    }
  }
}
