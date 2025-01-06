/*
 * MIT License
 *
 * Copyright (c) 2025 Team 86
 *
 * https://github.com/teamresistance
 *
 * More details provided in license files
 */

package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ChoreoWrap {
  public static List<String> getAllAutoNames() {
    File[] autoFiles = new File(Filesystem.getDeployDirectory(), "choreo").listFiles();

    if (autoFiles == null) {
      return new ArrayList<>();
    }

    return Stream.of(autoFiles)
        .filter(file -> !file.isDirectory())
        .map(File::getName)
        .filter(name -> name.endsWith(".traj"))
        .map(name -> name.substring(0, name.lastIndexOf(".")))
        .collect(Collectors.toList());
  }

  public static void addChoreoAutos(LoggedDashboardChooser<Command> autos, Drive drive) {

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    List<String> autoNames = getAllAutoNames();
    for (String name : autoNames) {
      ChoreoTrajectory traj = Choreo.getTrajectory(name);

      var thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      Command swerveCommand =
          Choreo.choreoSwerveCommand(
              traj,
              drive::getPose,
              new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
              new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
              thetaController,
              (ChassisSpeeds speeds) ->
                  drive.runVelocity(
                      new ChassisSpeeds(
                          speeds.vxMetersPerSecond,
                          speeds.vyMetersPerSecond,
                          speeds.omegaRadiansPerSecond)),
              () -> Constants.FIELD_MIRROR,
              drive);
      System.out.println("Choreo: " + name);
      autos.addOption(
          "Choreo: " + name,
          Commands.sequence(
              Commands.runOnce(() -> drive.resetOdometry(traj.getInitialPose())),
              swerveCommand,
              drive.run(
                  () ->
                      drive.runVelocity(
                          ChassisSpeeds.fromRobotRelativeSpeeds(
                              0,
                              0,
                              0,
                              isFlipped
                                  ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                  : drive.getRotation())))));
    }
  }
}
