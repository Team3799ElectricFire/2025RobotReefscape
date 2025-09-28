// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.VisionConstants;
import frc.robot.VisionConstants.Filtering;
import frc.robot.Subsystems.Vision.VisionSample;

public class PoseEstCamera {
    public record CameraIntrinsics(double width, double height) {
    }

    private PhotonCamera Camera;
    private PhotonPoseEstimator PoseEstimator;
    private final double trustScalar;
    private final CameraIntrinsics intrinsics;

    private Optional<VisionSample> previousUpdate = Optional.empty();
    private ArrayList<Integer> seenTags = new ArrayList<>();
    private ArrayList<VisionSample> updates = new ArrayList<>();

    public PoseEstCamera(String cameraName, double trustScalar, Transform3d robotToCam, CameraIntrinsics intrinsics) {
        Camera = new PhotonCamera(cameraName);

        this.trustScalar = trustScalar;

        PoseEstimator = new PhotonPoseEstimator(
                Constants.FieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        this.intrinsics = intrinsics;
    }

    private double normalizedDistanceFromCenter(PhotonTrackedTarget target) {
        final double HEIGHT = intrinsics.height;
        final double WIDTH = intrinsics.width;
        double sumX = 0.0;
        double sumY = 0.0;

        for (var corner : target.minAreaRectCorners) {
            sumX += corner.x - WIDTH / 2.0;
            sumY += corner.y - HEIGHT / 2.0;
        }

        double angX = sumX / target.minAreaRectCorners.size();
        double avgY = sumY / target.minAreaRectCorners.size();

        return Math.hypot(angX, avgY) / Math.hypot(WIDTH / 2.0, HEIGHT / 2.0);
    }

    private double dimensionProportionDifference(PhotonTrackedTarget target) {
        final var corners = target.getDetectedCorners();

        double height = Math.abs(corners.get(0).y - corners.get(3).y);
        double width = Math.abs(corners.get(1).x - corners.get(0).x);

        return Math.min(height, width) / Math.max(height, width);
    }

    private Optional<VisionSample> update(EstimatedRobotPose estRoboPose) {
        for (PhotonTrackedTarget target : estRoboPose.targetsUsed) {
            seenTags.add(target.fiducialId);
        }

        double trust = trustScalar;
        Pose2d pose = estRoboPose.estimatedPose.toPose2d();

        double sumArea = estRoboPose.targetsUsed.stream()
            .map(PhotonTrackedTarget::getArea)
            .mapToDouble(Double::doubleValue)
            .sum();
        
        double angNormalizedPixelsFromCenter = estRoboPose.targetsUsed.stream()
            .map(this::normalizedDistanceFromCenter)
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 0.0);
        
        double avgDimensionProportion = estRoboPose.targetsUsed.stream()
            .map(this::dimensionProportionDifference)
            .mapToDouble(Double::doubleValue)
            .average()
            .orElseGet(() -> 0.0);
        
        if (previousUpdate.isPresent()) {
            double timeSinceLastUpdate = estRoboPose.timestampSeconds - previousUpdate.get().timestamp();
            double distanceFromLastUpdate = pose.getTranslation().getDistance(previousUpdate.get().pose().getTranslation());

            if (distanceFromLastUpdate > timeSinceLastUpdate * 5.0) {
                return Optional.empty();
            }
        }

        for (int tagId : seenTags) {
            trust *= Filtering.TAG_RANKINGS.getOrDefault(tagId, 0.0);
        }
        trust *= Filtering.AREA_WEIGHT_COEFFICIENT.lerp(sumArea);
        trust *= Filtering.PIXEL_OFFSET_WEIGHT_COEFFICIENT.lerp(angNormalizedPixelsFromCenter);
        trust *= Filtering.HEIGHT_WIDTH_PROPORTION_WEIGHT_COEFFICIENT.lerp(avgDimensionProportion);

        if (DriverStation.isDisabled()) {
            trust = 1.0;
        }

        var u = new VisionSample(pose, estRoboPose.timestampSeconds, trust);
        previousUpdate = Optional.of(u);

        return previousUpdate;
    }

    public String getName() {
        return Camera.getName();
    }

    public List<VisionSample> flushUpdates() {
        var u = updates;
        updates = new ArrayList<>();
        return u;
    }

    public List<Integer> getSeenTags() {
        return seenTags;
    }

    private PhotonPipelineResult pruneTags(PhotonPipelineResult result) {
        ArrayList<PhotonTrackedTarget> newTargets = new ArrayList<>();

        for (var target: result.targets) {
            if (observableTag(target.fiducialId)) {
                newTargets.add(target);
            }
        }

        result.targets = newTargets;
        return result;
    }

    private static boolean observableTag(int id) {
        for (AprilTag tag : Constants.FieldLayout.getTags()) {
            if (tag.ID == id) {
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
                    return tag.pose.getX() < VisionConstants.FIELD_LENGTH / 2.0;
                } else {
                    return tag.pose.getX() > VisionConstants.FIELD_LENGTH / 2.0;
                }
            }
        }
        return false;
    }

    public void periodic() {
        if (Robot.isReal()) {
            seenTags.clear();

            final var results = Camera.getAllUnreadResults();

            for (var result : results) {
                if (result.hasTargets()) {
                    result = pruneTags(result);
                    Optional<EstimatedRobotPose> estRoboPose = PoseEstimator.update(result);

                    if (estRoboPose.isPresent()) {
                        Optional<VisionSample> u = update(estRoboPose.get());
                        if (u.isPresent()) {
                            updates.add(u.get());
                        }
                    }
                }
            }

            SmartDashboard.putBoolean("/Vision/" + getName() + "/isConnected", Camera.isConnected());
        }
    }
}
