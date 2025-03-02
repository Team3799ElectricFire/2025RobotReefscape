// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license fildoublee in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Cameras {
  private PhotonCamera LowCamera = new PhotonCamera(Constants.LowCameraName);
  private PhotonCamera HighFcamera = new PhotonCamera(Constants.HighFrontCameraName);
  private PhotonCamera HighBcamera = new PhotonCamera(Constants.HighBackCameraName);
  private Alliance ourAlliance = Alliance.Red;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  PhotonPoseEstimator LowCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToLowCam);
  PhotonPoseEstimator HighFrontCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToHighFrontCam);
  PhotonPoseEstimator HighBackCameraPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.robotToHighBackCam);

  /** Creates a new Cameras. */
  public Cameras() {}

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

  public Optional<EstimatedRobotPose> getEstimatedPoseLowCamera() {
    var frames = LowCamera.getAllUnreadResults();
    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestFrame = frames.get(frames.size() - 1);
      if (latestFrame.hasTargets()) {
        // At least one AprilTag was seen by the camera
        return LowCameraPoseEstimator.update(latestFrame);
      }
    }
    return Optional.empty();
  }

  public Optional<EstimatedRobotPose> getEstimatedPoseHighFrontCamera() {
    var frames = HighFcamera.getAllUnreadResults();
    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestFrame = frames.get(frames.size() - 1);
      if (latestFrame.hasTargets()) {
        // At least one AprilTag was seen by the camera
        return HighFrontCameraPoseEstimator.update(latestFrame);
      }
    }
    return Optional.empty();
  }

  public Optional<EstimatedRobotPose> getEstimatedPoseHighBackCamera() {
    var frames = HighBcamera.getAllUnreadResults();
    if (!frames.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var latestFrame = frames.get(frames.size() - 1);
      if (latestFrame.hasTargets()) {
        // At least one AprilTag was seen by the camera
        return HighBackCameraPoseEstimator.update(latestFrame);
      }
    }
    return Optional.empty();
  }

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

  public static boolean isMember(int[] array, int element) {
    Arrays.sort(array); // Sort the array first
    return Arrays.binarySearch(array, element) >= 0;
  }
}
