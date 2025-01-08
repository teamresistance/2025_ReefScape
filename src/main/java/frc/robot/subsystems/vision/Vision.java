/*
 * MIT License
 *
 * Copyright (c) 2025 Team 86
 *
 * https://github.com/teamresistance
 *
 * More details provided in license files
 */

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonCamera[] cams;
  private PhotonPoseEstimator[] poseEstimator;
  private Drive drive;

  public Vision(Transform3d[] cameraTransforms, String[] cameraNames, Drive drive) {
    this.drive = drive;
    cams = new PhotonCamera[cameraNames.length];
    poseEstimator = new PhotonPoseEstimator[cameraNames.length];

    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException thisIsDumb) {
      // Do nothing bc this is dumb // fair
    }

    for (int i = 0; i < cameraNames.length; i++) {
      try (PhotonCamera cam = new PhotonCamera(cameraNames[i])) {
        cams[i] = cam;
      } catch (Exception e) {
        DriverStation.reportError(
            "Exception while creating PhotonCamera: " + i + " " + e.getMessage(),
            e.getStackTrace());
      }
      poseEstimator[i] =
          new PhotonPoseEstimator(
              aprilTagFieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              cameraTransforms[i]);
      poseEstimator[i].setReferencePose(drive.getPose());
    }
  }

  public void periodic() {
    for (int i = 0; i < cams.length; i++) {
      PhotonPipelineResult result = cams[i].getLatestResult();
      if (result.hasTargets() && poseEstimator[i].getReferencePose() != null) {
        Object[] targets = result.getTargets().toArray();
        for (int j = 0; j < targets.length; j++) {
          Logger.recordOutput(
              "Vision/Targets/Cam" + i + "/results/Target" + j, targets[j].toString());
        }
        Optional<EstimatedRobotPose> update = poseEstimator[i].update(result);
        if (update.isPresent()) {
          Pose2d estimatedPose = update.get().estimatedPose.toPose2d();
          Logger.recordOutput("Vision/EstimatedPose/Cam" + i, estimatedPose.toString());
          drive.addVisionMeasurement(
              new Pose2d(estimatedPose.getTranslation(), estimatedPose.getRotation()),
              result.getTimestampSeconds());
        }
      }
    }
  }
}
