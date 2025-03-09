package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

public class PickupStationCmd extends Command {

    private static final Transform2d redUpperLeft = new Transform2d(15.85, 0.82, new Rotation2d(Units.degreesToRadians(-54.4)));
    private static final Transform2d redLowerLeft = new Transform2d(15.85, -0.82, new Rotation2d(Units.degreesToRadians(-54.4)));
    private static final Transform2d redUpperRight = new Transform2d(17.0, 0.82, new Rotation2d(Units.degreesToRadians(54.4)));
    private static final Transform2d redLowerRight = new Transform2d(17.0, -0.82, new Rotation2d(Units.degreesToRadians(54.4)));

    private static final Transform2d blueUpperLeft = new Transform2d(-15.85, 0.82, new Rotation2d(Units.degreesToRadians(54.4)));
    private static final Transform2d blueLowerLeft = new Transform2d(-15.85, -0.82, new Rotation2d(Units.degreesToRadians(54.4)));
    private static final Transform2d blueUpperRight = new Transform2d(-17.0, 0.82, new Rotation2d(Units.degreesToRadians(-54.4)));
    private static final Transform2d blueLowerRight = new Transform2d(-17.0, -0.82, new Rotation2d(Units.degreesToRadians(-54.4)));

    public PickupStationCmd(int id) {
        Transform2d selectedTransform;
        
        switch (id) {
            case 0 -> selectedTransform = redUpperLeft;
            case 1 -> selectedTransform = redLowerLeft;
            case 2 -> selectedTransform = redUpperRight;
            case 3 -> selectedTransform = redLowerRight;
            case 4 -> selectedTransform = blueUpperLeft;
            case 5 -> selectedTransform = blueLowerLeft;
            case 6 -> selectedTransform = blueUpperRight;
            case 7 -> selectedTransform = blueLowerRight;
            default -> throw new IllegalArgumentException("Invalid station ID: " + id);
        }

        RobotContainer.setStationTargetTransform(selectedTransform);
        RobotContainer.setStationOffsetTransform(new Transform2d(0.15, 0.0, new Rotation2d(0.0))); // Default offset
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
