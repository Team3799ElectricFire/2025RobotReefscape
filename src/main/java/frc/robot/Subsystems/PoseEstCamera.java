// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstCamera {
    private PhotonCamera Camera;
    private PhotonPoseEstimator PoseEstimator;
    private Matrix<N3, N1> CurStdDevs;
    private final Matrix<N3, N1> kSingleTagStdDevs, kMultiTagStdDevs;

    public PoseEstCamera(String cameraName, AprilTagFieldLayout aprilTagFieldLayout, Transform3d robotToCam,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
        Camera = new PhotonCamera(cameraName);

        PoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);
        PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        CurStdDevs = singleTagStdDevs;
        kSingleTagStdDevs = singleTagStdDevs;
        kMultiTagStdDevs = multiTagStdDevs;
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        // Initialize empty Optional variable representing the estimated robot pose
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        /*
         * For each frame camera has processed, use this camera's PhotonPoseEstimator
         * to update estimated robot pose and standard deviation of that estimated pose.
         * Each camera maintains its' own estimation.
         */
        for (var frame : Camera.getAllUnreadResults()) {
            visionEst = PoseEstimator.update(frame);
            updateStdDevs(visionEst, frame.getTargets());
        }
        return visionEst;
    }

    private void updateStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            CurStdDevs = kSingleTagStdDevs;
        } else {
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;
            for (var tgt : targets) {
                // Get the pose of the tag seen
                var tagPose = PoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

                // If tag seen is not part of the field, ignore it and go to the next target
                if (tagPose.isEmpty())
                    continue;

                numTags++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
                        estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                CurStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                CurStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3, N1> getEstStdDevs() {
        return CurStdDevs;
    }
    
    public void setDriverMode(boolean DriverMode) {
        Camera.setDriverMode(DriverMode);
    }
}
