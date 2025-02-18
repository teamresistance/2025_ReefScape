package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.GeomUtil;
import frc.robot.util.PolynomialRegression;
import frc.robot.util.TimestampedVisionUpdate;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  // Margin to avoid using poses that are outside the field.
  private static final double FIELD_BORDER_MARGIN = 0.5;
  // 3D poses of each camera on the robot.
  private static final Pose3d[] CAMERA_POSES = CameraPoses.cameraPoses;

  // The PhotonVision cameras.
  private final PhotonCamera[] cameras;

  // Scalars for tuning standard deviations of vision measurements.  Lower is more trusting.
  private final double singleTagStdDevScalar = 100.0; // Increased trust for single tag measurements

  private final double stdDevScalarAuto = 0.69420; // Standard deviation scalar during auto
  private final double thetaStdDevCoefficientAuto =
      0.1; // Coefficient for theta standard deviation during auto

  private final double stdDevScalarShooting = 0.2; // Standard deviation scalar during shooting
  private final double thetaStdDevCoefficientShooting =
      0.075; // Coefficient for theta standard deviation during shooting

  // Polynomial regression models for calculating standard deviations based on distance. MAY NEED TO
  // CALIBRATE
  private final PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
  private final PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
  private List<TimestampedVisionUpdate> visionUpdates;
  private Supplier<Pose2d> poseSupplier = Pose2d::new;

  /**
   * Creates a new Vision subsystem.
   *
   * @param cameras The PhotonVision cameras to use.
   * @throws IOException If the AprilTag field layout cannot be loaded.
   */
  public Vision(PhotonCamera... cameras) throws IOException {
    this.cameras = cameras;
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException ignored) {
      // The exception is caught and ignored because loading of the AprilTagFieldLayout can fail
      // while running in simulation, it is not necessary to have this
    }
  }

  /**
   * Sets the data interfaces for the Vision subsystem.
   *
   * @param poseSupplier The supplier for the current robot pose.
   * @param visionConsumer The consumer for processed vision updates.
   */
  public void setDataInterfaces(
      Supplier<Pose2d> poseSupplier, Consumer<List<TimestampedVisionUpdate>> visionConsumer) {
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = poseSupplier.get(); // Get the current estimated robot pose
    visionUpdates = new ArrayList<>(); // Create a new list to store vision updates

    double singleTagAdjustment = 1.0; // Initialize single tag adjustment factor
    if (Constants.TUNING_MODE)
      SingleTagAdjustment.updateLoggedTagAdjustments(); // Update adjustments if in tuning mode

    // Loop through all connected cameras
    for (int instanceIndex = 0; instanceIndex < cameras.length; instanceIndex++) {
      // Camera-specific variables
      Pose3d cameraPose = null; // Initialize to null. Only non-null if successfully calculated.
      Pose2d robotPose = null; // Same as above
      List<Pose3d> tagPose3ds =
          new ArrayList<>(); // List to store the 3D poses of detected AprilTags

      // Get all unread results from the camera
      List<PhotonPipelineResult> unprocessedResults = cameras[instanceIndex].getAllUnreadResults();
      if (unprocessedResults.isEmpty()) continue; // Skip this camera if there are no new results

      // Get the most recent result  (last in the list)
      PhotonPipelineResult unprocessedResult =
          unprocessedResults.get(unprocessedResults.size() - 1);

      // Log raw camera data for debugging
      Logger.recordOutput(
          "Photon/Raw Camera Data " + instanceIndex,
          SmartDashboard.getRaw(
              "photonvision/" + cameras[instanceIndex].getName() + "/rawBytes", new byte[] {}));

      // Skip this camera if there are no targets in the latest result
      if (!unprocessedResult.hasTargets()) {
        Logger.recordOutput("Photon/Tags Used " + instanceIndex, 0); // Log that no tags were used
        continue; // Move to the next camera
      }

      double timestamp = unprocessedResult.getTimestampSeconds(); // Get the timestamp of the result

      // Log camera data
      Logger.recordOutput(
          "Photon/Camera " + instanceIndex + " Has Targets", unprocessedResult.hasTargets());
      Logger.recordOutput(
          "Photon/Camera " + instanceIndex + "LatencyMS",
          unprocessedResult.metadata.getLatencyMillis());
      Logger.recordOutput("Photon/Camera " + instanceIndex + " Timestamp", timestamp);

      // Determine whether to use multi-tag or single-tag processing
      boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

      if (shouldUseMultiTag) {
        // Multi-tag processing

        // Get the estimated camera pose from the multi-tag result
        cameraPose =
            GeomUtil.transform3dToPose3d(
                unprocessedResult.getMultiTagResult().get().estimatedPose.best);

        // Calculate the robot pose from the camera pose and the camera's position on the robot
        robotPose =
            cameraPose
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();

        // Add the poses of all tags used in the multi-tag calculation to the list
        for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
          if (aprilTagFieldLayout.getTagPose(id).isPresent()) {
            tagPose3ds.add(aprilTagFieldLayout.getTagPose(id).get());
          }
        }

        Logger.recordOutput(
            "Photon/Camera Pose (Multi tag) " + instanceIndex, cameraPose); // Log the camera pose
      } else {
        // Single-tag processing

        // Get the best target from the result
        PhotonTrackedTarget target = unprocessedResult.targets.get(0);
        int tagId = target.getFiducialId();

        // Check to ensure the tag ID is valid.
        if (aprilTagFieldLayout.getTagPose(tagId).isEmpty()) {
          continue;
        }
        // Get the 3D pose of the detected tag
        Pose3d tagPos = aprilTagFieldLayout.getTagPose(tagId).get();

        // Calculate two possible camera poses based on the target's pose relative to the camera
        Pose3d cameraPose0 = tagPos.transformBy(target.getBestCameraToTarget().inverse());
        Pose3d cameraPose1 = tagPos.transformBy(target.getAlternateCameraToTarget().inverse());

        // Calculate two possible robot poses corresponding to the camera poses
        Pose2d robotPose0 =
            cameraPose0
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();
        Pose2d robotPose1 =
            cameraPose1
                .transformBy(GeomUtil.pose3dToTransform3d(CAMERA_POSES[instanceIndex]).inverse())
                .toPose2d();

        // Get the pose ambiguity.  Lower values are more reliable.
        double projectionError = target.getPoseAmbiguity();

        // Select the best camera and robot pose based on projection error and current robot
        // rotation
        if (projectionError < 0.15) {
          // If the ambiguity is low enough, assume the best pose is correct.
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else if (Math.abs(robotPose0.getRotation().minus(currentPose.getRotation()).getRadians())
            < Math.abs(robotPose1.getRotation().minus(currentPose.getRotation()).getRadians())) {
          // If the ambiguity is high, choose the pose whose rotation is closest to the current
          // estimated rotation.
          cameraPose = cameraPose0;
          robotPose = robotPose0;
        } else {
          // Choose the alternate pose if its rotation is closer to the current estimated rotation.
          cameraPose = cameraPose1;
          robotPose = robotPose1;
        }

        tagPose3ds.add(tagPos); // Add the tag pose to the list

        // Get the single-tag adjustment factor for the detected tag
        singleTagAdjustment = SingleTagAdjustment.getAdjustmentForTag(tagId);
        Logger.recordOutput(
            "Photon/Camera Pose (Single Tag) " + instanceIndex, cameraPose); // Log camera pose
      }

      // If the camera pose or robot pose could not be determined, skip to the next camera.
      if (cameraPose == null || robotPose == null) {
        continue;
      }

      // Check if the calculated robot pose is within the field boundaries
      if (robotPose.getX() < -FIELD_BORDER_MARGIN
          || robotPose.getX() > aprilTagFieldLayout.getFieldLength() + FIELD_BORDER_MARGIN
          || robotPose.getY() < -FIELD_BORDER_MARGIN
          || robotPose.getY() > aprilTagFieldLayout.getFieldWidth() + FIELD_BORDER_MARGIN) {
        continue; // Skip to next camera if robot pose is off field.
      }

      // Calculate the average distance to the detected tags
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPose3ds) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double avgDistance = totalDistance / tagPose3ds.size();

      // Calculate standard deviations for the vision measurement
      double xyStdDev;
      double thetaStdDev;

      if (shouldUseMultiTag) {
        // Multi-tag standard deviation calculation (simple, based on distance and number of tags)
        xyStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
        thetaStdDev = Math.pow(avgDistance, 2.0) / tagPose3ds.size();
      } else {
        // Single-tag standard deviation calculation (using polynomial regression models)
        xyStdDev = xyStdDevModel.predict(avgDistance);
        thetaStdDev = thetaStdDevModel.predict(avgDistance);
      }

      // Create a vision update with the calculated robot pose, timestamp, and standard deviations.
      if (shouldUseMultiTag) {
        // Multi-tag vision update (using multi-tag standard deviations)
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * xyStdDev,
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * xyStdDev,
                    stdDevScalarShooting * thetaStdDevCoefficientShooting * thetaStdDev)));
      } else {
        // Single-tag vision update (using single-tag standard deviations and adjustment factor)
        visionUpdates.add(
            new TimestampedVisionUpdate(
                robotPose,
                timestamp,
                VecBuilder.fill(
                    singleTagAdjustment * xyStdDev * stdDevScalarShooting,
                    singleTagAdjustment * xyStdDev * stdDevScalarShooting,
                    singleTagAdjustment * thetaStdDev * stdDevScalarShooting)));

        Logger.recordOutput("VisionData/" + instanceIndex, robotPose); // Log single-tag robot pose
        Logger.recordOutput(
            "Photon/Tags Used " + instanceIndex, tagPose3ds.size()); // Log number of tags used
      }
    }

    // Send all collected vision updates to the pose estimator
    visionConsumer.accept(visionUpdates);
  }
}
