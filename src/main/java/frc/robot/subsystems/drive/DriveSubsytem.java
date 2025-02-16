package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;


public class DriveSubsytem extends SubsystemBase {
    PhotonCamera FRcam = new PhotonCamera("front_right");
    PhotonCamera FLcam = new PhotonCamera("front_left");
    PhotonCamera BRcam = new PhotonCamera("back_right");
    PhotonCamera BLcam = new PhotonCamera("back_left");
    PhotonCamera FrontCam = new PhotonCamera("front");
    Optional<EstimatedRobotPose> pose = Optional.empty();
    AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public DriveSubsytem(){}

    //public Pose2d CameraToPose() {}
    //Move periodic code to its own method so each camera can have a pose made

    @Override
    public void periodic() {
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        Pose2d pose = photonPoseEstimator.get();
        SmartDashboard.putNumber("X", pose.getTranslation().getX());
        SmartDashboard.putNumber("Y", pose.getTranslation().getY());
        SmartDashboard.putNumber("Rotation", pose.getRotation().getDegrees());

        
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
}