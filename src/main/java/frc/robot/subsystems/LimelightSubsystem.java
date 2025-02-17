package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class LimelightSubsystem extends SubsystemBase {
    // BOTH APRILTAG BASED and REEF BRANCH BASED variables
    public Pose2d currentPose;
    public Pose2d alignedPose;
    public boolean isSeekingAlignment = false;
    public Object[] reefBranchCombinations = { "", "", 0 };
    String limelightName = RobotConstants.limelightName;
    // REEF BRANCH BASED variables
    double perceivedBranchWidthPixels;
    double forwardDistanceToBranchInches;
    double horizontalOffsetToBranchInches;

    public LimelightSubsystem() {
    }

    public void setSeekingAlignment(boolean changeTo) {
        this.isSeekingAlignment = changeTo;
    }

    public boolean isWithinErrorThreshold() {
        boolean inXThreshold = Math
                .abs(currentPose.getX() - alignedPose.getX()) < RobotConstants.kXdirectionErrorThresholdInches;
        boolean inYThreshold = Math
                .abs(currentPose.getX() - alignedPose.getX()) < RobotConstants.kYdirectionErrorThresholdInches;

        return inXThreshold && inYThreshold;
    }

    // APRILTAG BASED STRATEGY
    public int getNearestVisibleAprilTagID() { // Should only be called when target is visible
        return (int) LimelightHelpers.getT2DArray(limelightName)[9];
    }

    public double getAngleOfAprilTag(int id) {
        AprilTagFieldLayout field = AprilTagFieldLayout.loadField(
                AprilTagFields.k2025ReefscapeWelded);

        return field.getTagPose(id).get().getRotation().getZ();
    }

    public double[] getReefXY(String id) {
        if (!id.equals("M")) {
            switch (id) {
                case "A":
                    return FieldConstants.branchAposition;
                case "B":
                    return FieldConstants.branchBposition;
                case "C":
                    return FieldConstants.branchCposition;
                case "D":
                    return FieldConstants.branchDposition;
                case "E":
                    return FieldConstants.branchEposition;
                case "F":
                    return FieldConstants.branchFposition;
                case "G":
                    return FieldConstants.branchGposition;
                case "H":
                    return FieldConstants.branchHposition;
                case "I":
                    return FieldConstants.branchIposition;
                case "J":
                    return FieldConstants.branchJposition;
                case "K":
                    return FieldConstants.branchKposition;
                case "L":
                    return FieldConstants.branchLposition;
                default:
                    throw new IllegalArgumentException("Invalid branch ID: " + id);
            }
        } else {
            if (id.matches("A|B")) {
                return FieldConstants.reefABmidpoint;
            } else if (id.matches("C|D")) {
                return FieldConstants.reefCDmidpoint;
            } else if (id.matches("E|F")) {
                return FieldConstants.reefEFmidpoint;
            } else if (id.matches("G|H")) {
                return FieldConstants.reefGHmidpoint;
            } else if (id.matches("I|J")) {
                return FieldConstants.reefIJmidpoint;
            } else if (id.matches("K|L")) {
                return FieldConstants.reefKLmidpoint;
            } else {
                throw new IllegalArgumentException("Invalid branch ID: " + id);
            }
        }
    }

    /**
     * 
     * @param reefXY                   Array of length 2 that stores the x and y
     *                                 coordinates of the reef branch from the
     *                                 alliance origin.
     * @param aprilTagPoseAngleRadians The angle, in radians, of the AprilTag on the
     *                                 nearest side of the reef.
     * @param cameraToCenterOffset     Array of length 2 that stores the x and y
     *                                 offset of the camera with respect to the
     *                                 center of the robot.
     *                                 The +y direction is forward with the flipper
     *                                 side in front.
     *                                 Tje +x direction is rotated -90 degrees from
     *                                 the +y direction.
     * @param flipperToCenterOffset    Array of length 2 that stores the x and y
     *                                 offset of the flipper with respect to the
     *                                 center of the robot.
     *                                 The +y direction is forward with the flipper
     *                                 side in front.
     *                                 Tje +x direction is rotated -90 degrees from
     *                                 the +y direction.
     * @return The ideal pose of the robot to score on the branch expressed in
     *         reefXY.
     */
    public Pose2d getAlignedPose(double[] reefXY,
            double aprilTagPoseAngleRadians,
            double[] cameraToCenterOffset,
            double[] flipperToCenterOffset) {
        Rotation2d idealPoseAngle = new Rotation2d((aprilTagPoseAngleRadians + Math.PI) % (2 * Math.PI));

        double idealVector = cameraToCenterOffset[0]
                + FieldConstants.kReefBranchInsetInches
                - flipperToCenterOffset[0];

        double idealX = reefXY[0] + idealVector * Math.cos(aprilTagPoseAngleRadians);
        double idealY = reefXY[1] + idealVector * Math.sin(aprilTagPoseAngleRadians);

        return new Pose2d(idealX, idealY, idealPoseAngle);
    }

    // REEF BRANCH BASED STRATEGY
    public void setPBWP() {
        perceivedBranchWidthPixels = LimelightHelpers.getT2DArray(
                RobotConstants.limelightName)[13];
    }

    public void setFDTBI() {
        forwardDistanceToBranchInches = (RobotConstants.kLimelightWindowResolutionWidthPixels)
                * (FieldConstants.kReefBranchWidthInches)
                / (perceivedBranchWidthPixels
                        * Math.sin(
                                Math.toRadians(
                                        RobotConstants.kLimelightHorizontalFOVdegrees
                                                / 2))
                        * 2);
    }

    public void setHOTBI() {
        horizontalOffsetToBranchInches = forwardDistanceToBranchInches
                * Math.tan(
                        Math.toRadians(
                                LimelightHelpers.getTX(
                                        RobotConstants.limelightName)));
    }

    /**
     * 
     * @param reefPose                 Pose of the reef branch. ONLY USED IN THE
     *                                 REEF BRANCH BASED APPROACH.
     * @param aprilTagPoseAngleRadians The angle, in radians, of the AprilTag on the
     *                                 nearest side of the reef.
     * @param cameraToCenterOffset     Array of length 2 that stores the x and y
     *                                 offset of the camera with respect to the
     *                                 center of the robot.
     *                                 The +y direction is forward with the flipper
     *                                 side in front.
     *                                 Tje +x direction is rotated -90 degrees from
     *                                 the +y direction.
     * @param flipperToCenterOffset    Array of length 2 that stores the x and y
     *                                 offset of the flipper with respect to the
     *                                 center of the robot.
     *                                 The +y direction is forward with the flipper
     *                                 side in front.
     *                                 Tje +x direction is rotated -90 degrees from
     *                                 the +y direction.
     * @return The ideal pose of the robot to score on the branch expressed in
     *         reefXY.
     */
    public Pose2d getAlignedPose(Pose2d reefPose,
            Pose2d currentPose,
            double[] cameraToCenterOffset,
            double[] flipperToCenterOffset) {
        Rotation2d idealPoseAngle = new Rotation2d(reefPose.getRotation().getRadians());

        double idealVector = cameraToCenterOffset[0]
                + FieldConstants.kReefBranchInsetInches
                - flipperToCenterOffset[0];

        double idealX = currentPose.getX() + idealVector * Math.cos(reefPose.getRotation().getRadians());
        double idealY = currentPose.getY() + idealVector * Math.sin(reefPose.getRotation().getRadians());

        return new Pose2d(idealX, idealY, idealPoseAngle);
    }

    @Override
    public void periodic() {
        // APRILTAG BASED STRATEGY
        // We want to keep isSeekingAlignment as true until the robot's pose matches the
        // alignedPose.
        if (isSeekingAlignment && LimelightHelpers.getTV(limelightName)) {
            alignedPose = getAlignedPose(
                    getReefXY(reefBranchCombinations[0].toString()),
                    getAngleOfAprilTag(getNearestVisibleAprilTagID()),
                    RobotConstants.kCameraToCenterOffsetInches,
                    RobotConstants.kFlipperToCenterOffsetInches);
        }

        // REEF BRANCH BASED STRATEGY
        if (LimelightHelpers.getTV(limelightName)) {
            setPBWP();
            setFDTBI();
            setHOTBI();
        }
    }

    @Override
    public void simulationPeriodic() {
    }
}