// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.vision.SwerveModule;
public class SwerveAutonomousSubsystem extends SubsystemBase {
  Trajectory trajectory;
  Trajectory.State theStateOfTheTrajectory;
  Pose2d BlueATarget = new Pose2d(3.200, 3.306, new Rotation2d(Math.toRadians(0)));
  Pose2d BlueBTarget = new Pose2d(3.200, 2.959, new Rotation2d(Math.toRadians(0)));
  Pose2d BlueCTarget = new Pose2d(3.834, 3.125, new Rotation2d(Math.toRadians(60)));
  Pose2d BlueDTarget = new Pose2d(4.120, 3.86, new Rotation2d(Math.toRadians(60)));
  Pose2d BlueKTarget = new Pose2d(4.9261, 12.347, new Rotation2d(Math.toRadians(-60)));
  Pose2d BlueLTarget = new Pose2d(12.5308, 4.19, new Rotation2d(Math.toRadians(-60)));
  Pose2d currentPose2d;
  TrajectoryConfig trajectoryConfiguration = new TrajectoryConfig(1, 1);
  Spline.ControlVector controlVectorStart =
      new Spline.ControlVector(new double[] {0, 0, 0}, new double[] {0, 0, 0});
  Spline.ControlVector controlVectorEnd =
      new Spline.ControlVector(new double[] {0, 0, 0}, new double[] {0, 0, 0});
  Trajectory BlueAtrajectory;
  TrajectoryGenerator genny;
  QuinticHermiteSpline curvedPathway;
  Spline[] arrayOfSplines;
  SwerveModule frontLeft = new SwerveModule(1, 2, 3);
  SwerveModule frontRight = new SwerveModule(4, 5, 6);
  SwerveModule backLeft = new SwerveModule(7, 8, 9);
  SwerveModule backRight = new SwerveModule(10, 11, 12);
  double currentTime;
  SwerveModule[] swerveModules = new SwerveModule[4];
  public DriveSubsystem drive;
  {
  

    swerveModules[0] = frontLeft;
    swerveModules[1] = frontRight;
    swerveModules[2] = backLeft;
    swerveModules[3] = backRight;
  }

  /** Creates a new FlipperSubsystem. */
  public SwerveAutonomousSubsystem(DriveSubsystem d) {
    drive = d;
  }

  public void moveToTargetA() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueATarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueATarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  public void moveToTargetB() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueBTarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueBTarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  public void moveToTargetC() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueCTarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueCTarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  public void moveToTargetD() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueDTarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueDTarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  public void moveToTargetK() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueKTarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueKTarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  public void moveToTargetL() {
    final double startTime = System.currentTimeMillis() / 1000;
    double timeSince = currentTime - startTime;
    controlVectorStart.x = new double[] {currentPose2d.getX(), 1, 0};
    controlVectorStart.y = new double[] {currentPose2d.getY(), 1, 0};
    controlVectorEnd.x = new double[] {BlueLTarget.getX(), 0, 0};
    controlVectorEnd.y = new double[] {BlueLTarget.getY(), 0, 0};
    curvedPathway =
        new QuinticHermiteSpline(
            controlVectorStart.x, controlVectorStart.y, controlVectorEnd.x, controlVectorEnd.y);
    Spline[] arrayOfSplines = {curvedPathway};
    List<PoseWithCurvature> pathway = TrajectoryGenerator.splinePointsFromSplines(arrayOfSplines);
    List<Pose2d> twoDimensionalPoints = new ArrayList<>();
    for (PoseWithCurvature curvedPose : pathway) {
      twoDimensionalPoints.add(curvedPose.poseMeters);
    }
    theStateOfTheTrajectory = trajectory.sample(timeSince);
    trajectory =
        TrajectoryGenerator.generateTrajectory(twoDimensionalPoints, trajectoryConfiguration);
    swerveModules[0].setDesiredState(theStateOfTheTrajectory);
    swerveModules[1].setDesiredState(theStateOfTheTrajectory);
    swerveModules[2].setDesiredState(theStateOfTheTrajectory);
    swerveModules[3].setDesiredState(theStateOfTheTrajectory);
  }

  @Override
  public void periodic() {
    currentPose2d = drive.getPose();
    currentTime = System.currentTimeMillis() / 1000;
  }

  @Override
  public void simulationPeriodic() {
    currentPose2d = drive.getPose();
    currentTime = System.currentTimeMillis() / 1000;
  }
}
