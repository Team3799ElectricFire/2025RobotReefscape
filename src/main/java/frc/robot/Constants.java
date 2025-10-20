package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    // PID swerve constants
    public static final double SteeringPgain = 2.5;
    public static final double SteeringIgain = 0.001;
    public static final double SteeringDgain = 3; // Max derivative gain is 3 (according to REV hardware client)
    public static final double DrivingPgain = 0.1;
    public static final double DrivingIgain = 0;
    public static final double DrivingDgain = 0;
    public static final double DrivingFFgain = 1.0/565.0;

    // Conversion factors
    public static final double DriveMotorPositionFactor = 0.0521375063; // meters
    public static final double DriveMotorVelocityFactor = DriveMotorPositionFactor/60.0; // meters per sec
    public static final double SteerMotorPositionFactor = 2 * Math.PI; // radians
    public static final double ClimberPositionConversionFactor = 1.0/125.0; // revolutions of pulley
    public static final double WristPositionConversionFactor = 5.0/3.0; // degrees
    public static final double WristVelocityConversionFactor = WristPositionConversionFactor/60; // degrees per sec
    public static final double ElevatorPositionConversionFactor = 0.55889435962 * 2.0; // cm
    public static final double ElevatorVelocityConversionFactor = ElevatorPositionConversionFactor/60; // cm per sec

    // swerves limits
    public static final double kMinSpeedMetersPerSecond = 0.1;
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(19.3);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(360);
    public static final double kFrontRightChassisAngularOffset = 0.7922185; // CAN ID 7
    public static final double kFrontLeftChassisAngularOffset = 0.2272288; // CAN ID 1
    public static final double kBackRightChassisAngularOffset = 0.4184715; // CAN ID 5
    public static final double kBackLeftChassisAngularOffset = 0.9716559;  // CAN ID 3

    // CanbusID
    public static final int FrontRightDriveMotorID = 8;
    public static final int FrontRightSteerMotorID = 7;
    public static final int FrontLeftDriveMotorID = 2;
    public static final int FrontLeftSteerMotorID = 1;
    public static final int BackRightDriveMotorID = 6;
    public static final int BackRightSteerMotorID = 5;
    public static final int BackLeftDriveMotorID = 4;
    public static final int BackLeftSteerMotorID = 3;
    public static final int PidgeonID = 18;
    public static final int ElevatorLeftMotorID = 19;
    public static final int ElevatorRightMotorID = 20;
    public static final int ClimberMotorID = 15;
    public static final int WristLeftMotorID = 9;
    public static final int WristRightMotorID = 10;
    public static final int CoralLeftMotorID = 13;
    public static final int CoralRightMotorID = 14;
    public static final int AlgaeMotorID = 12;

    // Kinematics
    public static final double WheelBase = Units.inchesToMeters(23.75);
    public static final Translation2d FrontRightTranslation = new Translation2d(+WheelBase / 2, -WheelBase / 2);
    public static final Translation2d FrontLeftTranslation = new Translation2d(+WheelBase / 2, +WheelBase / 2);
    public static final Translation2d BackRightTranslation = new Translation2d(-WheelBase / 2, -WheelBase / 2);
    public static final Translation2d BackLeftTranslation = new Translation2d(-WheelBase / 2, +WheelBase / 2);
    public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            FrontRightTranslation,
            FrontLeftTranslation,
            BackRightTranslation,
            BackLeftTranslation);

    // Elevator PID
    public static final double ElevatorKP = 0.1;
    public static final double ElevatorKI = 0;
    public static final double ElevatorKD = 3;
    public static final double ElevatorKS = 0;
    public static final double ElevatorKG = 0;
    public static final double ElevatorKV = 0;
    public static final double ElevatorMotionMaxVelocity = 55; // cm per second
    public static final double ElevatorMotionMaxAcceleration = 55; // cm per second^2
    public static final double ElevatorMotionAllowedError = 1; // cm

    // Wrist PID
    public static final double WristKP = 0.75;
    public static final double WristKI = 0;
    public static final double WristKD = 3;
    public static final double WristKS = 0;
    public static final double WristKG = 0;
    public static final double WristKV = 0;
    public static final double WristMotionMaxVelocity = 90; // degrees per second
    public static final double WristMotionMaxAcceleration = 90; // degrees per second^2
    public static final double WristMotionAllowedError = 1; // degrees

    // Motor Speeds
    public static final double HighSpeedMultiple = 1.0;
    public static final double LowSpeedMultiple = 0.50;
    public static final double L2SpeedMultiple = 1.0;
    public static final double L3SpeedMultiple = 0.5;
    public static final double L4SpeedMultiple = 0.3;
    public static final double minThumbstickMagnitude = 0.1;
    public static final double ElevatorSpeed = 0.25;
    public static final double AlgaeIntakeSpeed = 0.3;
    public static final double AlgaeOutakeSpeed = 1.0;
    public static final double AlgaeHold = 0.10;
    public static final double CoralIntakeSpeed = 0.30;
    public static final double CoralSecureSpeed = -0.15;
    public static final double CoralOutakeSpeed = 0.45;
    public static final double CoralShelfSpeed = 0.10;
    public static final double ClimberDownSpeed = -1.0;
    public static final double ClimberUpSpeed = 1.0;
    public static final double WristSpeed = 0.25;

    // Position Set Points
    public static final double ElevatorLevel1 = 0;// centimeters
    public static final double ElevatorLevel2 = 27.5;// centimeters, previously 30- JB 3/9/25
    public static final double ElevatorLevel3 = 68;// centimeters, previously 69.5- JB 3/9/25
    public static final double ElevatorLevel4 = 131;// centimeters
    public static final double ElevatorSoftLimMax = 137;//cm
    public static final double ElevatorSoftLimCoral = 42;//cm (highest safe height with coral in elevator's way)
    public static final double ElevatorSoftLimMin = 0;//cm 
    public static final double WristStart = 40;// degree
    public static final double WristScore = 21;//degree
    public static final double WristTravel = 21;//degree
    public static final double WristFloorPickUp = -15;// degree
    public static final double WristReefPickUp = 20;//degree
    public static final double WristSoftLimMax = 40;//degree
    public static final double WristSoftLimMin = -17;//degree

    // Driving Constants
    public static final double panRateOfChangeLimit = 10.0; // Translation Drive Demand Rate-of-Change Limit, units/sec
    public static final double rotRateOfChangeLimit = 10.0; // Rotation Drive Demand Rate-of-Change Limit, units/sec
    public static final double teleAngleHoldFactor = 0.1; // Teleop heading maintaining P-gain, 1/degrees
    public static final double teleCameraHoldFactor = 0.10; // Teleop vision targeting P-gain, 1/degrees

    // Path Planner
    public static final double MassKG = 57;
    public static final double MOI = 6.883;
    public static final ModuleConfig SwerveConfig = new ModuleConfig(
            Units.inchesToMeters(2),
            kMaxSpeedMetersPerSecond,
            1.0, 
            DCMotor.getNeoVortex(1).withReduction(6.12),
            50,
            1);
    public static final RobotConfig ROBOTCONFIG = new RobotConfig(
            MassKG,
            MOI,
            SwerveConfig,
            FrontRightTranslation, FrontLeftTranslation, BackRightTranslation, BackLeftTranslation);
    public static final PIDConstants TranslationPIDconstants = new PIDConstants(
            30,
            0.75,
            0.0);
    public static final PIDConstants RotationPIDconstants = new PIDConstants(
            13.0,
            0.20,
            0.0);


   // Cameras
   public static final String LowCameraName = "LowCamera";
   public static final String HighFrontCameraName = "HighFcamera";
   public static final String HighBackCameraName = "HighBcamera";

   public static final int BlueProcessorTag = 16;
   public static final int RedProcessorTag = 3;
   public static final int[] BlueReef = {17,18,19,20,21,22};
   public static final int[] RedReef = {6,7,8,9,10,11};
   public static final int[] BlueCoralstation = {12,13};
   public static final int[] RedCoralstation = {1,2};

   public static final Transform3d robotToLowCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(5.857), Units.inchesToMeters(8.831), Units.inchesToMeters(11.75)), 
        new Rotation3d(0,0,Units.degreesToRadians(-26)));
   public static final Transform3d robotToHighFrontCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(5.177), Units.inchesToMeters(8.721), Units.inchesToMeters(39.426)), 
        new Rotation3d(0,Units.degreesToRadians(-40),0));
   public static final Transform3d robotToHighBackCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(-0.648), Units.inchesToMeters(9.239), Units.inchesToMeters(38.239)),
        new Rotation3d(Units.degreesToRadians(180),Units.degreesToRadians(-140),0));
   
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5,3);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,0.5,1);
  public static final double CameraTrustMaxSpeed = 2.0;
  public static final double CameraTrustMultSpeed = 2.0;
  public static final double CameraTrustMaxRot = Math.PI * 2.0 / 5.0;
  public static final double CameraTrustMultRot = 2.0;

  public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.1,0.1,0.1);
  public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(1,1,1);


  // Driver Assist
  public static final double X_REEF_ALIGNMENT_P = 2.0; // TODO tune these
  public static final double Y_REEF_ALIGNMENT_P = 2.0;
  public static final double ROT_REEF_ALIGNMENT_P = 0.0075;

  public static final double X_SETPOINT_REEF_ALIGNMENT = -1.2827;  // Vertical pose [m]
  public static final double X_TOLERANCE_REEF_ALIGNMENT = Units.inchesToMeters(1.0);
  public static final double Y_SETPOINT_REEF_ALIGNMENT = Units.inchesToMeters(-6.5);  // Horizontal pose [m]
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = Units.inchesToMeters(1.0);
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation [deg]
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 3;

  public static final double POSE_VALIDATION_TIME = 0.3;
  public static final double REEF_ALIGN_MAX_TIME = 2.0;

  public static final double kFieldLength = 17.548;
  public static final double kFieldWidth = 8.052;
  public static final double kReefCenterToWallDistance = 0.781;
  public static final Translation2d kReefCenterBlue = new Translation2d(4.489, kFieldWidth/2.0);
  public static final Translation2d kReefCenterRed = new Translation2d(kFieldLength-4.489, kFieldWidth/2.0);
  public static final double kFacingReefTolerance = 1.0;
  public static final AprilTagFieldLayout FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
} 
