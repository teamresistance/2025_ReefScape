package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class CameraPoses {
    public static final Pose3d[] cameraPoses =
            new Pose3d[] {
                    // Front Left
                    new Pose3d(
                            new Translation3d(0.270, 0.334, 0.267), // Right camera translation (X, Y, Z)
                            new Rotation3d(0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(45))),
                    // Front Right
                    new Pose3d(
                            new Translation3d(0.270, -0.334, 0.267), // Left camera translation (X, Y, Z)
                            new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-12.63),
                                    Units.degreesToRadians(-45))), // in radians btw
                    // Back Left
                    new Pose3d(
                            new Translation3d(-0.27, 0.334, 0.267), // Right camera translation (X, Y, Z)
                            new Rotation3d(0.0, Units.degreesToRadians(-12.63), Units.degreesToRadians(45 + 90))),
                    // Back Right
                    new Pose3d(
                            new Translation3d(-0.27, -0.334, 0.267), // Left camera translation (X, Y, Z)
                            new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-12.63),
                                    Units.degreesToRadians(-45 - 90))), // in radians btw

                    // front_center
                    new Pose3d(
                            new Translation3d(
                                    0.31, -0.03, Units.inchesToMeters(7.7)), // Left camera translation (X, Y, Z)
                            new Rotation3d(
                                    0.0, Units.degreesToRadians(-13.5), Units.degreesToRadians(0))), // in radians btw
            };
}
