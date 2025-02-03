// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Arrays;

import org.photonvision.PhotonCamera;

public class Cameras extends SubsystemBase {
  private PhotonCamera LowCamera = new PhotonCamera(Constants.LowCameraName);
  private PhotonCamera HighFcamera = new PhotonCamera(Constants.HighFrontCameraName);
  private PhotonCamera HighBcamera = new PhotonCamera(Constants.HighBackCameraName);
  private Alliance ourAlliance = Alliance.Red;

  /** Creates a new Cameras. */
  public Cameras() {
  }

  @Override
  public void periodic() {

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

  public double getAngleToProcessor() {
    boolean targetVisible = false;
    double targetYaw = 0.0;

    var results = HighFcamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (ourAlliance == Alliance.Red && target.getFiducialId() == Constants.RedProcessorTag) {
            // Found tag 3
            targetYaw = target.getYaw();
            targetVisible = true;
          } else if (ourAlliance == Alliance.Blue && target.getFiducialId() == Constants.BlueProcessorTag) {
            // Found Tag 16, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }

    if (targetVisible) {
      return targetYaw;
    } else {
      return -999;
    }
  }

  public double getAngleToCoralStation() {
    boolean targetVisible = false;
    double targetYaw = 0.0;

    var results = HighBcamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (ourAlliance == Alliance.Red && isMember(Constants.RedCoralstation, target.getFiducialId())) {
            // Found tag 1 or 2
            targetYaw = target.getYaw();
            targetVisible = true;
          } else if (ourAlliance == Alliance.Blue && isMember(Constants.BlueCoralstation, target.getFiducialId())) {
            // Found Tag 12 or 13, record its information
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }

    if (targetVisible) {
      return targetYaw;
    } else {
      return -999;
    }
  }

  public double getAngleToReef(){
    boolean targetVisible = false;
    double targetYaw = 0.0;

    var results = LowCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (ourAlliance == Alliance.Red && isMember(Constants.RedReef, target.getFiducialId())) {
            // Found tag 6, 7, 8, 9, 10, or 11 
            targetYaw = target.getYaw();
            targetVisible = true;
          } else if (ourAlliance == Alliance.Blue && isMember(Constants.BlueReef, target.getFiducialId())) {
            // Found Tag 17, 18, 19, 20, 21 or 22 , record its information
            targetYaw = target.getYaw();
            targetVisible = true;
          }
        }
      }
    }

    if (targetVisible) {
      return targetYaw;
    } else {
      return -999;
    }
  }

  public static boolean isMember(int[] array, int element) {
    Arrays.sort(array); // Sort the array first
    return Arrays.binarySearch(array, element) >= 0;
  }
}
