// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Annotations.Log;

public class Drivetrain extends SubsystemBase implements Logged {
  // Swerve modules
  private SwerveModule FrontRightModule = new SwerveModule(
      Constants.FrontRightDriveMotorID,
      Constants.FrontRightSteerMotorID,
      Constants.kFrontRightChassisAngularOffset);
  private SwerveModule FrontLeftModule = new SwerveModule(
      Constants.FrontLeftDriveMotorID,
      Constants.FrontLeftSteerMotorID,
      Constants.kFrontLeftChassisAngularOffset);
  private SwerveModule BackRightModule = new SwerveModule(
      Constants.BackRightDriveMotorID,
      Constants.BackRightSteerMotorID,
      Constants.kBackRightChassisAngularOffset);
  private SwerveModule BackLeftModule = new SwerveModule(
      Constants.BackLeftDriveMotorID,
      Constants.BackLeftSteerMotorID,
      Constants.kBackLeftChassisAngularOffset);

  // Gyro sensor
  private Canandgyro Pidgey = new Canandgyro(Constants.PidgeonID);
  
  // Pose Estimator for tracking robot pose
  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
    Constants.kDriveKinematics, 
    Pidgey.getRotation2d(), 
    getModulePositions(),
    new Pose2d(),
    Constants.kStateStdDevs,
    Constants.kVisionStdDevs);

  private PoseEstCamera BottomCam = new PoseEstCamera(Constants.LowCameraName, Constants.FieldLayout, Constants.robotToLowCam, Constants.kSingleTagStdDevs, Constants.kMultiTagStdDevs);
  private PoseEstCamera HighFCam = new PoseEstCamera(Constants.HighFrontCameraName, Constants.FieldLayout, Constants.robotToHighFrontCam, Constants.kSingleTagStdDevs, Constants.kMultiTagStdDevs);
  private PoseEstCamera HighBCam = new PoseEstCamera(Constants.HighBackCameraName, Constants.FieldLayout, Constants.robotToHighBackCam, Constants.kSingleTagStdDevs, Constants.kMultiTagStdDevs);

  private boolean _DriveRobotRelative = false;
  private double SpeedMultiple = Constants.LowSpeedMultiple;
  private Translation2d RotationCenter = new Translation2d();
  
  private Alliance ourAlliance = Alliance.Red;
  @Log
  public Pose2d reefReference = Pose2d.kZero;
  @Log
  private boolean facingReef = false;
  @Log
  private double wallDistance = 0.0;

  public boolean TurningToReef = false;
  @Log
  public String ActiveCommand;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Autobuilder
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            Constants.TranslationPIDconstants,
            Constants.RotationPIDconstants),
        Constants.ROBOTCONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Update pose estimator with info from gyro and swerve modules
    poseEstimator.update(Pidgey.getRotation2d(), getModulePositions());

    // Update pose estimator with info from cameras
    UpdatePoseWithCameras();

    // Track reef's location
    Translation2d reefCenter = (ourAlliance == Alliance.Blue) ? Constants.kReefCenterBlue : Constants.kReefCenterRed;
    Translation2d reefTranslation = getPose().getTranslation().minus(reefCenter);
    Rotation2d reefAngle = new Rotation2d(
        Math.floor(
            reefCenter.minus(getPose().getTranslation()).getAngle().plus(
                new Rotation2d(Math.PI / 6.0)).getRadians() /
                (Math.PI / 3.0))
            * (Math.PI / 3.0));
    reefReference = new Pose2d(reefCenter, reefAngle);
    facingReef = epsilonEquals(0.0, reefAngle.minus(getPose().getRotation()).getRadians(),
        Constants.kFacingReefTolerance);
    wallDistance = Math.max(0.0,
        reefAngle.rotateBy(Rotation2d.kPi).minus(reefTranslation.getAngle()).getCos() * reefTranslation.getNorm()
            - Constants.kReefCenterToWallDistance);

    var active = this.getCurrentCommand();
    if (active != null) {
      ActiveCommand = active.getName();
    }
  }

  public void setAlliance(Alliance color) {
    ourAlliance = color;
  }

  @Log
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Pidgey.getRotation2d(),
        getModulePositions(),
        pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
      double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(getModuleState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);

    //UptadePoseWithCameras(); // TODO enable here to only use pose estimation from cameras in Auto, not teleop
  }

  private void UpdatePoseWithCameras() {
    var EstimatedLowPose = BottomCam.getEstimatedPose(getRobotRelativeSpeeds());
    EstimatedLowPose.ifPresent(
      est -> {
        //System.out.println("X: " + Double.toString(est.estimatedPose.toPose2d().getX()) + " | Y: " + Double.toString(est.estimatedPose.toPose2d().getY()) + " | R: " + Double.toString(est.estimatedPose.s));
       // System.out.println(est.estimatedPose.toPose2d().toString());
        var estStdDevs = BottomCam.getEstStdDevs();
        addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
    );

    var EstimatedHighFrontPose = HighFCam.getEstimatedPose();
    EstimatedHighFrontPose.ifPresent(
      est -> {
        var estStdDevs = HighFCam.getEstStdDevs();
        addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
    );

    var EstimatedHighBackPose = HighBCam.getEstimatedPose();
    EstimatedHighBackPose.ifPresent(
      est -> {
        var estStdDevs = HighBCam.getEstStdDevs();
        addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
    );
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    SmartDashboard.putNumber("Vx", xVelocityMetersPerSecond);
    SmartDashboard.putNumber("Vy", yVelocityMetersPerSecond);
    SmartDashboard.putNumber("rot", rotationRadiansPerSecond);

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationRadiansPerSecond, Pidgey.getRotation2d());

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }

  public void setX() {
    FrontRightModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    FrontLeftModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    BackRightModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    BackLeftModule.setDesiredStateNoRestrictions(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  public void stop() {
    FrontRightModule.stop();
    FrontLeftModule.stop();
    BackRightModule.stop();
    BackLeftModule.stop();
  }

  @Log
  public SwerveModuleState[] getModuleState() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = FrontRightModule.getState();
    states[1] = FrontLeftModule.getState();
    states[2] = BackRightModule.getState();
    states[3] = BackLeftModule.getState();

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        FrontRightModule.getPosition(),
        FrontLeftModule.getPosition(),
        BackRightModule.getPosition(),
        BackLeftModule.getPosition()
    };
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);
    FrontRightModule.setDesiredState(desiredStates[0]);
    FrontLeftModule.setDesiredState(desiredStates[1]);
    BackRightModule.setDesiredState(desiredStates[2]);
    BackLeftModule.setDesiredState(desiredStates[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(desiredSpeeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a
    // new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);
  }

  public void resetEncoders() {
    FrontRightModule.resetEncoders();
    FrontLeftModule.resetEncoders();
    BackRightModule.resetEncoders();
    BackLeftModule.resetEncoders();
  }

  public void setHeading(double newAngleDegrees) {
    Pidgey.setYaw(newAngleDegrees);
  }

  public void zeroHeading() {
    Pidgey.setYaw(0.0);
  }

  public Command ZeroHeadingCommand(){
    return runOnce(() -> {
      zeroHeading();
    }).asProxy();
  }
  
  @Log
  public double getHeading() {
    return Pidgey.getRotation2d().getDegrees();
  }

  public double getPitch() {
    return Pidgey.getPitch();
  }

  public double getRoll() {
    return Pidgey.getRoll();
  }

  public void setDriveRobotRelative() {
    this._DriveRobotRelative = true;
  }

  public void setDriveFieldRelative() {
    this._DriveRobotRelative = false;
  }

  public void toggleDriveRobotRelative() {
    this._DriveRobotRelative = !this._DriveRobotRelative;
  }

  public Command toggleDriveRobotRelativeCommand(){ 
    return runOnce(() -> {
      toggleDriveRobotRelative();
    }).asProxy();
  }

  public Command setDriveSpeeCommand(double newSpeed) {
    return runOnce(() -> {
      SpeedMultiple = newSpeed;
    }).asProxy();
  }

  public boolean getDriveRobotRelative() {
    return this._DriveRobotRelative;
  }

  public void setHighSpeed() {
    SpeedMultiple = Constants.HighSpeedMultiple;
  }

  public Command setHgihSpeedCommand() {
    return runOnce(() -> {
      setHighSpeed();
    }).asProxy();
  }

  public void setLowSpeed() {
    SpeedMultiple = Constants.LowSpeedMultiple;
  }

  public Command setLowSpeedCommand() {
    return runOnce(() -> {
      setLowSpeed();
    }).asProxy();
  }

  public void toggleHiLoSpeed() {
    if (SpeedMultiple == Constants.LowSpeedMultiple) {
      setHighSpeed();
    } else {
      setLowSpeed();
    }
  }

  public void setRotationCenterGamePiece(Translation2d newCenter) {
    RotationCenter = newCenter;
  }

  public void resetRotationCenterRobot() {
    RotationCenter = new Translation2d();
  }

  public boolean epsilonEquals(double a, double b, double epsilon) {
    return (a-epsilon <= b) && (a+epsilon >= b);
  }

  public Command faceReefCommand() {
    return runOnce(
    () -> {this.TurningToReef = true;});
  }

  public Command stopLookingAtReef() {
    return runOnce(
      () -> {this.TurningToReef = false;});
  }
}
