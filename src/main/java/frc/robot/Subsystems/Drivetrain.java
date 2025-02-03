// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
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
  private WPI_PigeonIMU Pidgey = new WPI_PigeonIMU(Constants.PidgeonID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(
      Constants.kDriveKinematics,
      Pidgey.getRotation2d(),
      new SwerveModulePosition[] {
          FrontRightModule.getPosition(),
          FrontLeftModule.getPosition(),
          BackRightModule.getPosition(),
          BackLeftModule.getPosition()
      });

  private boolean _DriveRobotRelative = true;
  private double SpeedMultiple = Constants.LowSpeedMultiple;
  private Translation2d RotationCenter = new Translation2d();
  // private int targetFiducialID = AprilTagIDs.NoTarget; // ID number of AprilTag
  // we are aiming at (0 means no target, aka camera off)

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // PigeonIMUConfiguration config = new PigeonIMUConfiguration();

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
    Odometry.update(Pidgey.getRotation2d(), new SwerveModulePosition[] {
        FrontRightModule.getPosition(),
        FrontLeftModule.getPosition(),
        BackRightModule.getPosition(),
        BackLeftModule.getPosition()
    });
  }

  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    Odometry.resetPosition(
        Pidgey.getRotation2d(),
        new SwerveModulePosition[] {
            FrontRightModule.getPosition(),
            FrontLeftModule.getPosition(),
            BackRightModule.getPosition(),
            BackLeftModule.getPosition()
        },
        pose);
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
  }

  public void driveRobotRelative(double xSpeed, double ySpeed, double rot) {
    double xVelocityMetersPerSecond = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double yVelocityMetersPerSecond = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotationRadiansPerSecond = rot * Constants.kMaxAngularSpeed;

    ChassisSpeeds speeds = new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond,
        rotationRadiansPerSecond);

    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(speeds, RotationCenter);

    // desaturateWheelSpeed() is changing the value of moduleStates, not returning a
    // new variable
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
    Pidgey.reset();
  }

  public double getHeading() {
    return Pidgey.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return Pidgey.getRate();
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

  /*
   * public void setTargetNone() {
   * targetFiducialID = AprilTagIDs.NoTarget;
   * }
   * public void setTargetAmp() {
   * //Check Alliance Color :
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/
   * alliancecolor.html
   * Optional<Alliance> ally = DriverStation.getAlliance();
   * if (ally.isPresent()) {
   * if (ally.get() == Alliance.Red) {
   * targetFiducialID = AprilTagIDs.RedAmp;
   * }
   * if (ally.get() == Alliance.Blue) {
   * targetFiducialID = AprilTagIDs.BlueAmp;
   * }
   * } else {
   * targetFiducialID = AprilTagIDs.NoTarget;
   * }
   * }
   * public void setTargetSpeaker() {
   * //Check Alliance Color:
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/
   * alliancecolor.html
   * Optional<Alliance> ally = DriverStation.getAlliance();
   * if (ally.isPresent()) {
   * if (ally.get() == Alliance.Red) {
   * targetFiducialID = AprilTagIDs.RedSpeaker;
   * }
   * if (ally.get() == Alliance.Blue) {
   * targetFiducialID = AprilTagIDs.BlueSpeaker;
   * }
   * } else {
   * targetFiducialID = AprilTagIDs.NoTarget;
   * }
   * }
   * public int getTargetID() {
   * return targetFiducialID;
   * }
   * public Command TargetAmpCommand() {
   * return this.startEnd(this::setTargetAmp, this::setTargetNone).asProxy();
   * }
   * public Command TargetSpeakerCommand() {
   * return this.startEnd(this::setTargetSpeaker, this::setTargetNone).asProxy();
   * }
   */
}
