package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    // PID swerve constants
    public static final double SteeringPgain = 0;
    public static final double SteeringIgain = 0;
    public static final double SteeringDgain = 0;
    public static final double DrivingPgain = 0;
    public static final double DrivingIgain = 0;
    public static final double DrivingDgain = 0;

    // Conversion factors
    public static final double DriveMotorVelocityFactor = 8.6895843e-4;
    public static final double SteerMotorPositionFactor = 2 * Math.PI;
    public static final double ClimberPositionConversionFactor = 1.38888888888;
    public static final double WristPositionConversionFactor = 1.66666666666;
    public static final double ElevatorPositionConversionFactor = 0.55889435962;

    // swerves limits
    public static final double kMinSpeedMetersPerSecond = 0.1;
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(19.3);
    public static final double kMaxAngularSpeed = Units.degreesToRadians(360);
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = Math.PI;

    // CanbusID
    public static final int FrontRightDriveMotorID = 1;
    public static final int FrontRightSteerMotorID = 2;
    public static final int FrontLeftDriveMotorID = 3;
    public static final int FrontLeftSteerMotorID = 4;
    public static final int BackRightDriveMotorID = 5;
    public static final int BackRightSteerMotorID = 6;
    public static final int BackLeftDriveMotorID = 7;
    public static final int BackLeftSteerMotorID = 8;
    public static final int PidgeonID = 9;
    public static final int ElevatorLeftMotorID = 10;
    public static final int ElevatorRightMotorID = 11;
    public static final int ClimberLeftMotorID = 12;
    public static final int ClimberRightMotorID = 13;
    public static final int WristLeftMotorID = 14;
    public static final int WristRightMotorID = 15;
    public static final int CoralLeftMotorID = 16;
    public static final int CoralRightMotorID = 17;
    public static final int AlgaeLeftMotorID = 18;
    public static final int AlgaeRightMotorID = 19;

    // DIO Channels
    public static final int CoralTopSwitch = 3;
    public static final int CoralBottomSwitch = 4;
    public static final int ClimberHomeSwitch = 5;

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
    public static final double ElevatorKP = 0;
    public static final double ElevatorKI = 0;
    public static final double ElevatorKD = 0;
    public static final double ElevatorKS = 0;
    public static final double ElevatorKG = 0;
    public static final double ElevatorKV = 0;

    // Wrist PID
    public static final double WristKP = 0;
    public static final double WristKI = 0;
    public static final double WristKD = 0;
    public static final double WristKS = 0;
    public static final double WristKG = 0;
    public static final double WristKV = 0;

    // Motor Speeds
    public static final double TurboSpeedMultiple = 0.90;
    public static final double HighSpeedMultiple = 1.0;
    public static final double LowSpeedMultiple = 0.50;
    public static final double minThumbstickMagnitude = 0.1;
    public static final double ElevatorSpeed = 0.5;
    public static final double AlgaeIntakeSpeed = 0.5;
    public static final double AlgaeOutakeSpeed = 1.0;
    public static final double AlgaeHold = 0.25;
    public static final double CoralIntakeSpeed = 0.7;
    public static final double CoralOutakeSpeed = 0.8;
    public static final double CoralShelfSpeed = 0.9;
    public static final double ClimberDownSpeed = -1.0;
    public static final double ClimberUpSpeed = 1.0;
    public static final double WristSpeed = 0.8;


    // Position Set Points
    public static final double ElevatorLevel1 = 0;// centimeters
    public static final double ElevatorLevel2 = 35;// centimeters
    public static final double ElevatorLevel3 = 75;// centimeters
    public static final double ElevatorLevel4 = 137;// centimeters
    public static final double ElevatorSoftLimMax = 137;//cm
    public static final double ElevatorSoftLimCoral = 42;//cm (highest safe height with coral in elevator's way)
    public static final double ElevatorSoftLimMin = 0;//cm 
    public static final double WristStart = 40;// degree
    public static final double WristScore = 25;//degree
    public static final double WristTravel = 25;//degree
    public static final double WristPickUp = -15;// degree
    public static final double WristSoftLimMax = 40;//degree
    public static final double WristSoftLimMin = -15;//degree

    // Path Planner
    public static final double MassKG = 30;
    public static final double MOI = 20;
    public static final ModuleConfig SwerveConfig = new ModuleConfig(
            Units.inchesToMeters(2),
            kMaxSpeedMetersPerSecond,
            1.0, // TODO DOUBLE CHECK
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

} 
