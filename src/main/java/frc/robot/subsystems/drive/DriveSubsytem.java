package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public class DriveSubsytem extends SubsystemBase {
    PhotonCamera FRcam = new PhotonCamera("front_right");
    PhotonCamera FLcam = new PhotonCamera("front_left");
    PhotonCamera BRcam = new PhotonCamera("back_right");
    PhotonCamera BLcam = new PhotonCamera("back_left");
    PhotonCamera FrontCam = new PhotonCamera("front");
    PhotonCamera BackCam = new PhotonCamera("back");
    Optional<EstimatedRobotPose> pose = Optional.empty();
    AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public DriveSubsytem(){}

    public Pose2d CameraToPose(PhotonCamera camera){
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        PhotonPipelineResult PPLR = camera.getLatestResult();
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(PPLR);
        Pose3d pose3d = new Pose3d();
        if (estimatedPose.isPresent()) {
            pose3d = estimatedPose.get().estimatedPose;
        }
        Pose2d pose2d = pose3d.toPose2d();
        return pose2d;
    }
    public Pose2d AveragePose (Pose2d pose1, Pose2d pose2, Pose2d pose3, Pose2d pose4, Pose2d pose5, Pose2d pose6){
        double avgX = (pose1.getX() + pose2.getX() + pose3.getX() + pose4.getX() + pose5.getX() + pose6.getX()) / 6;
        double avgY = (pose1.getY() + pose2.getY() + pose3.getY() + pose4.getY() + pose5.getY() + pose6.getY()) / 6;
        double sumCos = pose1.getRotation().getCos() + pose2.getRotation().getCos() + pose3.getRotation().getCos()
         + pose4.getRotation().getCos() + pose5.getRotation().getCos() + pose6.getRotation().getCos();
        double sumSin = pose1.getRotation().getSin() + pose2.getRotation().getSin() + pose3.getRotation().getSin()
         + pose4.getRotation().getSin() + pose5.getRotation().getSin() + pose6.getRotation().getSin();
         Rotation2d avgRot = new Rotation2d(Math.atan2(sumSin, sumCos));
        Pose2d averagePose = new Pose2d(avgX, avgY, avgRot);
        return averagePose;
    }

    @Override
    public void periodic() {}
}