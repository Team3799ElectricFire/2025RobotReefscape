// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private SparkMax LeftMotor = new SparkMax(Constants.WristLeftMotorID, MotorType.kBrushless);
  private SparkMax RightMotor = new SparkMax(Constants.WristRightMotorID, MotorType.kBrushless);
  private ArmFeedforward FeedForward = new ArmFeedforward(Constants.WristKS, Constants.WristKG, Constants.WristKV);
  private SparkClosedLoopController PID;
  private SparkLimitSwitch HomeSwitch = RightMotor.getForwardLimitSwitch();

  /** Creates a new Wrist. */
  public Wrist() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.follow(Constants.WristRightMotorID, true);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.encoder
        .positionConversionFactor(Constants.WristPositionConversionFactor)
        .velocityConversionFactor(Constants.WristVelocityConversionFactor);
    rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.WristKP, Constants.WristKI, Constants.WristKD)
        .outputRange(-1, 1);
    rightConfig.closedLoop.maxMotion
        .maxVelocity(Constants.WristMotionMaxVelocity)
        .maxAcceleration(Constants.WristMotionMaxAcceleration)
        .allowedClosedLoopError(Constants.WristMotionAllowedError);
    rightConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);
        rightConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.WristSoftLimMax)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.WristSoftLimMin);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    PID = RightMotor.getClosedLoopController();

    if (AtHome()) {
      HomeEncoder();
    }
    //SetReference(getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WRIST ANGLE", getAngle());
    SmartDashboard.putBoolean("WRIST HOME", AtHome());
  }

  public void WristUp() {
    RightMotor.set(Constants.WristSpeed);
  }

  public void WristDown() {
    RightMotor.set(Constants.WristSpeed * -1);
  }

  public void WristStop() {
    RightMotor.set(0);
  }

  public void SetReference(double newSetPoint) {
    newSetPoint = Math.min(newSetPoint, Constants.WristSoftLimMax);
    newSetPoint = Math.max(newSetPoint, Constants.WristSoftLimMin);
    PID.setReference(newSetPoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward.calculate(newSetPoint, 0));
  }

  public boolean AtHome() {
    return HomeSwitch.isPressed();
  }

  public Command GoToPositionCommand(double newSetPoint) {
    return runOnce(() -> {
      SetReference(newSetPoint);
    });
  }

  public void HomeEncoder() {
    RightMotor.getEncoder().setPosition(Constants.WristStart);
  }

  public Command HomeEncoderCommand() {
    return runOnce(() -> {
      HomeEncoder();
    }).ignoringDisable(true);
  }

  public double getAngle() {
    return RightMotor.getEncoder().getPosition();
  }
}
