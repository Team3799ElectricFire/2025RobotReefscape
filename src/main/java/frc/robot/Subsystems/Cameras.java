// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license fildoublee in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Cameras {
  private Alliance ourAlliance = Alliance.Red;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Low Front Camera
  private PhotonCamera LowCamera = new PhotonCamera(Constants.LowCameraName);
  private PhotonPoseEstimator LowCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToLowCam);
  private Matrix<N3,N1> LowCameraCurStdDevs = Constants.kSingleTagStdDevs;

  // High Front Camera
  private PhotonCamera HighFcamera = new PhotonCamera(Constants.HighFrontCameraName);
  private PhotonPoseEstimator HighFrontCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToHighFrontCam);
  private Matrix<N3,N1> HighFrontCameraCurStdDevs = Constants.kSingleTagStdDevs;

  // High Back Camera
  private PhotonCamera HighBcamera = new PhotonCamera(Constants.HighBackCameraName);
  private PhotonPoseEstimator HighBackCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToHighBackCam);
  private Matrix<N3,N1> HighBackCameraCurStdDevs = Constants.kSingleTagStdDevs;

  /** Creates a new Cameras. */
  public Cameras() {
    // Fallback Strategy Settings
    LowCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    HighFrontCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    HighBackCameraPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public void setLowDriverMode(boolean newMode) {
    LowCamera.setDriverMode(newMode);
  }

  public void setHighFDriverMode(boolean newMode) {
    HighFcamera.setDriverMode(newMode);
  }

  public void setHighBDriverMode(boolean newMode) {
    HighBcamera.setDriverMode(newMode);
  }

  public boolean getLowDriverMode() {
    return LowCamera.getDriverMode();
  }

  public boolean getHighFDriverMode() {
    return HighFcamera.getDriverMode();
  }

  public boolean getHighBDriverMode() {
    return HighBcamera.getDriverMode();
  }

  public void setAlliance(Alliance color) {
    ourAlliance = color;
  }

  // region LowCamera
  public Optional<EstimatedRobotPose> getEstimatedPoseLowCamera() {
    // Initialize empty Optional variable representing the estimated robot pose
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    /*
     * For each frame camera has processed, use this camera's PhotonPoseEstimator
     * to update estimated robot pose and standard deviation of that estimated pose.
     * Each camera maintains its' own estimation.
     */
    for (var frame : LowCamera.getAllUnreadResults()) {
      visionEst = LowCameraPoseEstimator.update(frame);
      updateLowCamStdDevs(visionEst, frame.getTargets());
    }
    return visionEst;
  }

  private void updateLowCamStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      LowCameraCurStdDevs = Constants.kSingleTagStdDevs;
    } else {
      var estStdDevs = Constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        // Get the pose of the tag seen
        var tagPose = LowCameraPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        // If tag seen is not part of the field, ignore it and go to the next target
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
          estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        LowCameraCurStdDevs = Constants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = Constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          LowCameraCurStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getLowCameraEstStdDevs() {
    return LowCameraCurStdDevs;
  }
  // endregion

  // region HighFrontCamera
  public Optional<EstimatedRobotPose> getEstimatedPoseHighFrontCamera() {
    // Initialize empty Optional variable representing the estimated robot pose
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    /*
     * For each frame camera has processed, use this camera's PhotonPoseEstimator
     * to update estimated robot pose and standard deviation of that estimated pose.
     * Each camera maintains its' own estimation.
     */
    for (var frame : HighFcamera.getAllUnreadResults()) {
      visionEst = HighFrontCameraPoseEstimator.update(frame);
      updateHighFrontCamStdDevs(visionEst, frame.getTargets());
    }
    return visionEst;
  }

  private void updateHighFrontCamStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      HighFrontCameraCurStdDevs = Constants.kSingleTagStdDevs;
    } else {
      var estStdDevs = Constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        // Get the pose of the tag seen
        var tagPose = HighFrontCameraPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        // If tag seen is not part of the field, ignore it and go to the next target
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
          estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        HighFrontCameraCurStdDevs = Constants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = Constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          HighFrontCameraCurStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getHighFrontCameraEstStdDevs() {
    return HighFrontCameraCurStdDevs;
  }
  // endregion

  // region HighBackCamera
  public Optional<EstimatedRobotPose> getEstimatedPoseHighBackCamera() {
    // Initialize empty Optional variable representing the estimated robot pose
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    /*
     * For each frame camera has processed, use this camera's PhotonPoseEstimator
     * to update estimated robot pose and standard deviation of that estimated pose.
     * Each camera maintains its' own estimation.
     */
    for (var frame : HighBcamera.getAllUnreadResults()) {
      visionEst = HighBackCameraPoseEstimator.update(frame);
      updateHighBackCamStdDevs(visionEst, frame.getTargets());
    }
    return visionEst;
  }

  private void updateHighBackCamStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      HighBackCameraCurStdDevs = Constants.kSingleTagStdDevs;
    } else {
      var estStdDevs = Constants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        // Get the pose of the tag seen
        var tagPose = HighBackCameraPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        // If tag seen is not part of the field, ignore it and go to the next target
        if (tagPose.isEmpty()) continue;

        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation().getDistance(
          estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        HighBackCameraCurStdDevs = Constants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = Constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          HighBackCameraCurStdDevs = estStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getHighBackCameraEstStdDevs() {
    return HighBackCameraCurStdDevs;
  }
  // endregion

  public Optional<Double> getAngleToProcessor() {
    var frames = HighFcamera.getAllUnreadResults();
    PhotonTrackedTarget target = null;
    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestFrame = frames.get(frames.size() - 1);
      if (latestFrame.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var detection : latestFrame.getTargets()) {
          if (ourAlliance == Alliance.Red && detection.getFiducialId() == Constants.RedProcessorTag) {
            // Found tag 3
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          } else if (ourAlliance == Alliance.Blue && detection.getFiducialId() == Constants.BlueProcessorTag) {
            // Found Tag 16, record its information
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          }
        }
      }
    }
    if (target != null) {
      return Optional.of(target.getSkew());
    } else {
      return Optional.empty();
    }
  }

  public Optional<Double> getAngleToCoralStation() {
    var frames = HighBcamera.getAllUnreadResults();
    PhotonTrackedTarget target = null;

    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestFrame = frames.get(frames.size() - 1);
      if (latestFrame.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var detection : latestFrame.getTargets()) {
          if (ourAlliance == Alliance.Red && isMember(Constants.RedCoralstation, detection.getFiducialId())) {
            // Found tag 1 or 2
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          } else if (ourAlliance == Alliance.Blue && isMember(Constants.BlueCoralstation, detection.getFiducialId())) {
            // Found Tag 12 or 13, record its information
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          }
        }
      }
    }

    if (target != null) {
      return Optional.of(target.getSkew());
    } else {
      return Optional.empty();
    }
  }

  public Optional<Double> getAngleToReef() {
    var frames = LowCamera.getAllUnreadResults();
    PhotonTrackedTarget target = null;

    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestframe = frames.get(frames.size() - 1);
      if (latestframe.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var detection : latestframe.getTargets()) {
          if (ourAlliance == Alliance.Red && isMember(Constants.RedReef, detection.getFiducialId())) {
            // Found tag 6, 7, 8, 9, 10, or 11
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          } else if (ourAlliance == Alliance.Blue && isMember(Constants.BlueReef, detection.getFiducialId())) {
            // Found Tag 17, 18, 19, 20, 21 or 22 , record its information
            if (target == null || detection.area > target.area) {
              target = detection;
            }
          }
        }
      }
    }

    if (target != null) {
      return Optional.of(target.getSkew());
    } else {
      return Optional.empty();
    }
  }

  public Optional<PhotonTrackedTarget> getLowCameraTag() {
    Optional<PhotonTrackedTarget> target = Optional.empty();
    var ourReef = (ourAlliance == Alliance.Red) ? Constants.RedReef : Constants.BlueReef;

    var frames = LowCamera.getAllUnreadResults();
    if (!frames.isEmpty()) {
      var latestFrame = frames.get(frames.size() - 1);
      for (var detection : latestFrame.getTargets()) {
        if (isMember(ourReef, detection.getFiducialId())) {
          if (target.isEmpty() || detection.area > target.get().area) {
            target = Optional.of(detection);
          }
        }
      }
    }
    return target;
  }

  public static boolean isMember(int[] array, int element) {
    Arrays.sort(array); // Sort the array first
    return Arrays.binarySearch(array, element) >= 0;
  }
}
