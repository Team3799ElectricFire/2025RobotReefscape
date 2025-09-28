package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.PoseEstCamera.CameraIntrinsics;
import frc.robot.lib.LerpTable;

public class VisionConstants {
    public record CameraConfig(String name, double trustScalar, Transform3d transform, CameraIntrinsics intrinsics) {
    }

    public static final double FIELD_LENGTH = Units.inchesToMeters(690.876);

    public static final CameraConfig[] CONFIGS = {
        new CameraConfig(
            "Left",
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(5.857),
                    Units.inchesToMeters(8.831),
                    Units.inchesToMeters(11.75)
                ),
                new Rotation3d(0,0,Units.degreesToRadians(-26.0))
            ),
            new CameraIntrinsics(1280, 800)
        ),
        new CameraConfig(
            "Right", 
            1.0,
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(5.857),
                    Units.inchesToMeters(-8.831),
                    Units.inchesToMeters(11.75)
                ),
                new Rotation3d(0,0,Units.degreesToRadians(26.0))
            ),
            new CameraIntrinsics(1280, 800)
        ),
    };

    public static final class Filtering {
        public static final LerpTable HEIGHT_WIDTH_PROPORTION_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.25, 0.0),
                new LerpTable.LerpTableEntry(0.7, 0.9),
                new LerpTable.LerpTableEntry(1.0, 1.0));

        public static final LerpTable AREA_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 0.0),
                new LerpTable.LerpTableEntry(0.2, 0.35),
                new LerpTable.LerpTableEntry(1.0, 0.45),
                new LerpTable.LerpTableEntry(4.0, 0.70),
                new LerpTable.LerpTableEntry(7.5, 1.0));

        public static final LerpTable PIXEL_OFFSET_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(0.2, 1.0),
                new LerpTable.LerpTableEntry(0.65, 0.75),
                new LerpTable.LerpTableEntry(1.0, 0.35));

        public static final LerpTable LINEAR_VELOCITY_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(2.5, 0.8),
                new LerpTable.LerpTableEntry(5.0, 0.1));

        public static final LerpTable ANGULAR_VELOCITY_WEIGHT_COEFFICIENT = new LerpTable(
                new LerpTable.LerpTableEntry(0.0, 1.0),
                new LerpTable.LerpTableEntry(7.0, 0.65),
                new LerpTable.LerpTableEntry(12.0, 0.0));

        public static final HashMap<Integer, Double> TAG_RANKINGS = new HashMap<>() {
            {
                put(1, 0.0); // CORAL STATION
                put(2, 0.0); // CORAL STATION
                put(3, 0.0); // PROCESSOR
                put(4, 0.0); // BARGE
                put(5, 0.0); // BARGE
                put(6, 1.0); // REEF
                put(7, 1.0); // REEF
                put(8, 1.0); // REEF
                put(9, 1.0); // REEF
                put(10, 1.0); // REEF
                put(11, 1.0); // REEF
                put(12, 0.0); // CORAL STATION
                put(13, 0.0); // CORAL STATION
                put(14, 0.0); // BARGE
                put(15, 0.0); // BARGE
                put(16, 0.0); // PROCESSOR
                put(17, 1.0); // REEF
                put(18, 1.0); // REEF
                put(19, 1.0); // REEF
                put(20, 1.0); // REEF
                put(21, 1.0); // REEF
                put(22, 1.0); // REEF
            }
        };
    }
}
