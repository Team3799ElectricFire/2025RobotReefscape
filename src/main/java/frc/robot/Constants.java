package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final double DriveMotorVelocityFactor = 8.6895843e-4;
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
    public static final double kBackRightChassisAngularOffset = 0.3619109; // CAN ID 5
    public static final double kBackLeftChassisAngularOffset = 0.2692254;  // CAN ID 3

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
    public static final double TurboSpeedMultiple = 0.90;
    public static final double HighSpeedMultiple = 1.0;
    public static final double LowSpeedMultiple = 0.50;
    public static final double minThumbstickMagnitude = 0.1;
    public static final double ElevatorSpeed = 0.25;
    public static final double AlgaeIntakeSpeed = 0.3;
    public static final double AlgaeOutakeSpeed = 1.0;
    public static final double AlgaeHold = 0.10;
    public static final double CoralIntakeSpeed = 0.15;
    public static final double CoralSecureSpeed = -0.10;
    public static final double CoralOutakeSpeed = 0.35;
    public static final double CoralShelfSpeed = 0.10;
    public static final double ClimberDownSpeed = -1.0;
    public static final double ClimberUpSpeed = 1.0;
    public static final double WristSpeed = 0.6;

    // Position Set Points
    public static final double ElevatorLevel1 = 0;// centimeters
    public static final double ElevatorLevel2 = 30;// centimeters
    public static final double ElevatorLevel3 = 69.5;// centimeters
    public static final double ElevatorLevel4 = 135;// centimeters
    public static final double ElevatorSoftLimMax = 137;//cm
    public static final double ElevatorSoftLimCoral = 42;//cm (highest safe height with coral in elevator's way)
    public static final double ElevatorSoftLimMin = 0;//cm 
    public static final double WristStart = 40;// degree
    public static final double WristScore = 25;//degree
    public static final double WristTravel = 25;//degree
    public static final double WristPickUp = -13;// degree
    public static final double WristSoftLimMax = 40;//degree
    public static final double WristSoftLimMin = -15;//degree

    // Driving Constants
    public static final double panRateOfChangeLimit = 10.0; // Translation Drive Demand Rate-of-Change Limit, units/sec
    public static final double rotRateOfChangeLimit = 10.0; // Rotation Drive Demand Rate-of-Change Limit, units/sec
    public static final double teleAngleHoldFactor = 0.1; // Teleop heading maintaining P-gain, 1/degrees
    public static final double teleCameraHoldFactor = 0.010; // Teleop vision targeting P-gain, 1/degrees

    // Path Planner
    public static final double MassKG = 30;
    public static final double MOI = 20;
    public static final ModuleConfig SwerveConfig = new ModuleConfig(
            Units.inchesToMeters(2),
            kMaxSpeedMetersPerSecond,
            1.0, 
            DCMotor.getNeoVortex(1),
            50,
            1);
    public static final RobotConfig ROBOTCONFIG = new RobotConfig(
            MassKG,
            MOI,
            SwerveConfig,
            FrontRightTranslation, FrontLeftTranslation, BackRightTranslation, BackLeftTranslation);
    public static final PIDConstants TranslationPIDconstants = new PIDConstants(
            5.0,
            0.0,
            0.0); // TODO tune this PID
    public static final PIDConstants RotationPIDconstants = new PIDConstants(
            5.0,
            0.0,
            0.0);  // TODO tune this PID


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
   public static final Transform3d robotToLowCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
   public static final Transform3d robotToHighFrontCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
   public static final Transform3d robotToHighBackCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
   // TODO confirm camera locations

} 
