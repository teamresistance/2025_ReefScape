package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.Optional;

public class PickupStationCmd extends Command {

  public static Optional<DriverStation.Alliance> ally;
  private final int id;

  private static final Transform2d redUpperLeft =
      new Transform2d(15.9, 1.64, new Rotation2d(Units.degreesToRadians(-54.4)));
  private static final Transform2d redLowerLeft =
      new Transform2d(16.819, 1.391, new Rotation2d(Units.degreesToRadians(-54.4)));
  private static final Transform2d redUpperRight =
      new Transform2d(15.9, 7.3, new Rotation2d(Units.degreesToRadians(54.4)));
  private static final Transform2d redLowerRight =
      new Transform2d(16.819, 6.625, new Rotation2d(Units.degreesToRadians(54.4)));

  private static final Transform2d blueUpperLeft =
      new Transform2d(1.645, 7.360, new Rotation2d(Units.degreesToRadians(125.600)));
  private static final Transform2d blueLowerLeft =
      new Transform2d(0.698, 6.625, new Rotation2d(Units.degreesToRadians(125.600)));
  private static final Transform2d blueUpperRight =
      new Transform2d(0.698, 1.391, new Rotation2d(Units.degreesToRadians(-125.600)));
  private static final Transform2d blueLowerRight =
      new Transform2d(1.645, 0.698, new Rotation2d(Units.degreesToRadians(-125.000)));

  public PickupStationCmd(int id) {
    this.id = id;
  }

  @Override
  public void initialize() {
    if (ally.isPresent()) {
      DriverStation.Alliance team = ally.get();
      Transform2d newTransform;
      switch (id) {
        case 0 ->
            newTransform = (team == DriverStation.Alliance.Red) ? redUpperLeft : blueUpperLeft;
        case 1 ->
            newTransform = (team == DriverStation.Alliance.Red) ? redLowerLeft : blueLowerLeft;
        case 2 ->
            newTransform = (team == DriverStation.Alliance.Red) ? redUpperRight : blueUpperRight;
        case 3 ->
            newTransform = (team == DriverStation.Alliance.Red) ? redLowerRight : blueLowerRight;
        default -> throw new IllegalArgumentException("Invalid station ID: " + id);
      }
      RobotContainer.setStationTargetTransform(newTransform);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
