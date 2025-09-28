// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.VisionConstants.CameraConfig;
import frc.robot.VisionConstants.Filtering;
import monologue.Logged;

public class Vision extends SubsystemBase implements Logged{
  private final PoseEstCamera[] cameras;

  private final HashSet<Integer> seenTags = new HashSet<>();
  private final ChassisSpeeds speeds = new ChassisSpeeds();
  private final ArrayList<VisionSample> samples = new ArrayList<>();

  public record VisionSample(Pose2d pose, double timestamp, double weight) implements StructSerializable {
    private static final VisionSample kEmpty = new VisionSample(Pose2d.kZero, 0.0, 1.0);
    
    public static VisionSample empty() {
      return kEmpty;
    }
  }

  public static PoseEstCamera[] camerasFromConfigs(CameraConfig... configs) {
    PoseEstCamera[] cameras = new PoseEstCamera[configs.length];

    for (int i = 0; i < configs.length; i++) {
      CameraConfig config = configs[i];
      cameras[i] = new PoseEstCamera(
        config.name(),
        config.trustScalar(),
        config.transform(),
        config.intrinsics()
      );
    }
    return cameras;
  }

  public Vision(PoseEstCamera... cameras) {
    this.cameras = cameras;
  }

  public void updateSpeeds(ChassisSpeeds speeds) {
    this.speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    this.speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
  }

  private Optional<VisionSample> gaugeWeight(final VisionSample update) {
    double weight = update.weight;

    weight *= Filtering.LINEAR_VELOCITY_WEIGHT_COEFFICIENT.lerp(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    weight *= Filtering.ANGULAR_VELOCITY_WEIGHT_COEFFICIENT.lerp(speeds.omegaRadiansPerSecond);

    if (DriverStation.isDisabled()) {
      weight = 1.0;
    }

    return Optional.of(new VisionSample(update.pose(), update.timestamp(), weight));
  }

  public List<VisionSample> flushSamples() {
    List<VisionSample> outList = new ArrayList<>();
    outList.addAll(samples);
    samples.clear();
    return outList;
  }

  @Override
  public void periodic() {
    for (final PoseEstCamera camera : cameras) {
      
      try {
        camera.periodic();
      } catch (Exception e) {
        DriverStation.reportError("Error in Camera " + camera.getName(), e.getStackTrace());
      }

      camera.flushUpdates().stream()
        .map(this::gaugeWeight)
        .filter(Optional::isPresent)
        .map(Optional::get)
        .forEach(
          sample -> {
            samples.add(sample);
            // GlobalField.setObject(camera.getName() + "Camera", sample.pose());
          }
        );
      
      seenTags.addAll(camera.getSeenTags());
      
      Pose2d[] tagLoc = seenTags.stream()
        .map(i -> Constants.FieldLayout.getTagPose(i))
        .filter(Optional::isPresent)
        .map(Optional::get)
        .map(Pose3d::toPose2d)
        .toArray(Pose2d[]::new);

      // GlobalField.setObject("SeenTags", tagLoc);

      seenTags.clear();
    }
  }
}
