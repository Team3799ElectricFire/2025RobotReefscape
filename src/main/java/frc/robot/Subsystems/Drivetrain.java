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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
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
    new SwerveModulePosition[] {
      FrontRightModule.getPosition(),
      FrontLeftModule.getPosition(),
      BackRightModule.getPosition(),
      BackLeftModule.getPosition()
  }, new Pose2d());

  public Cameras eyeballCameras = new Cameras();
  private boolean _DriveRobotRelative = true;
  private double SpeedMultiple = Constants.LowSpeedMultiple;
  private Translation2d RotationCenter = new Translation2d();
  private final StructArrayPublisher<SwerveModuleState> publisher;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

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
    poseEstimator.update(Pidgey.getRotation2d(), new SwerveModulePosition[] {
        FrontRightModule.getPosition(),
        FrontLeftModule.getPosition(),
        BackRightModule.getPosition(),
        BackLeftModule.getPosition()
    });

    publisher.set(getModuleState());

    printDS();
  }

  private void printDS() {
    SmartDashboard.putNumber("GYRO ANGLE", getHeading());
    SmartDashboard.putNumber("FL HEADING", FrontLeftModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("FR HEADING", FrontRightModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("BL HEADING", BackLeftModule.getState().angle.getDegrees());
    SmartDashboard.putNumber("BR HEADING", BackRightModule.getState().angle.getDegrees());
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
        Pidgey.getRotation2d(),
        new SwerveModulePosition[] {
            FrontRightModule.getPosition(),
            FrontLeftModule.getPosition(),
            BackRightModule.getPosition(),
            BackLeftModule.getPosition()
        },
        pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public Command TurnOnBackCameraCommand(){
    return runOnce(() -> {eyeballCameras.setHighBDriverMode(false);});
  }

  public Command TurnOffBackCameraCommand(){
    return runOnce(() -> {eyeballCameras.setHighBDriverMode(true);});
  }

  public Command TurnOnHighFCameraCommand(){
    return runOnce(() -> {eyeballCameras.setHighFDriverMode(false);});
  }

  public Command TurnOffHighFCameraCommand(){
    return runOnce(() -> {eyeballCameras.setHighFDriverMode(true);});
  }

  public Command TurnOnLowCameraCommand(){
    return runOnce(() -> {eyeballCameras.setLowDriverMode(false);});
  }

  public Command TurnOffLowCameraCommand(){
    return runOnce(() -> {eyeballCameras.setLowDriverMode(true);});
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(getModuleState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a
    // new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    FrontLeftModule.setDesiredState(moduleStates[1]);
    BackRightModule.setDesiredState(moduleStates[2]);
    BackLeftModule.setDesiredState(moduleStates[3]);

    /*var EstimatedPose = eyeballCameras.getEstimatedPoseLowCamera();
    if (EstimatedPose.isPresent()){
      var result = EstimatedPose.get();
      addVisionMeasurement(result.estimatedPose.toPose2d(), result.timestampSeconds, null);
    }*/
    
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    SmartDashboard.putNumber("Vx", xVelocityMetersPerSecond);
    SmartDashboard.putNumber("Vy", yVelocityMetersPerSecond);
    SmartDashboard.putNumber("rot", rotationRadiansPerSecond);

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond,
        rotationRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a
    // new variable
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kMaxSpeedMetersPerSecond * SpeedMultiple);

    FrontRightModule.setDesiredState(moduleStates[0]);
    SmartDashboard.putNumber("FR ANGLE SP", moduleStates[0].angle.getDegrees());

    FrontLeftModule.setDesiredState(moduleStates[1]);
    SmartDashboard.putNumber("FL ANGLE SP", moduleStates[1].angle.getDegrees());

    BackRightModule.setDesiredState(moduleStates[2]);
    SmartDashboard.putNumber("BR ANGLE SP", moduleStates[2].angle.getDegrees());

    BackLeftModule.setDesiredState(moduleStates[3]);
    SmartDashboard.putNumber("BL ANGLE SP", moduleStates[3].angle.getDegrees());
  }

  public void driveFieldRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond,
        rotationRadiansPerSecond, Pidgey.getRotation2d());

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a
    // new variable
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

  public SwerveModuleState[] getModuleState() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = FrontRightModule.getState();
    states[1] = FrontLeftModule.getState();
    states[2] = BackRightModule.getState();
    states[3] = BackLeftModule.getState();

    return states;
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
    return runOnce(() -> {zeroHeading();});
  }
  
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
    // System.out.println("Set Robot Relative");
  }

  public void setDriveFieldRelative() {
    this._DriveRobotRelative = false;
    // System.out.println("Set Field Relative");
  }

  public void toggleDriveRobotRelative() {
    this._DriveRobotRelative = !this._DriveRobotRelative;
  }

  public boolean getDriveRobotRelative() {
    return this._DriveRobotRelative;
  }

  public void setTurboSpeed() {
    SpeedMultiple = Constants.TurboSpeedMultiple;
  }

  public void setHighSpeed() {
    SpeedMultiple = Constants.HighSpeedMultiple;
  }

  public void setLowSpeed() {
    SpeedMultiple = Constants.LowSpeedMultiple;
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

}
