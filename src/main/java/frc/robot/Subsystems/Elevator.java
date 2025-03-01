// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkFlex LeftMotor = new SparkFlex(Constants.ElevatorLeftMotorID, MotorType.kBrushless);
  private SparkFlex RightMotor = new SparkFlex(Constants.ElevatorRightMotorID, MotorType.kBrushless);
  private ElevatorFeedforward FeedForward = new ElevatorFeedforward(Constants.ElevatorKS, Constants.ElevatorKG,
      Constants.ElevatorKV);
  private SparkClosedLoopController PIDController;
  private double SoftMax = Constants.ElevatorSoftLimMax;
  private TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.ElevatorMotionMaxVelocity, Constants.ElevatorMotionMaxAcceleration));
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  /** Creates a new Elevator. */
  public Elevator() {
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.follow(Constants.ElevatorRightMotorID, true);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.inverted(true);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.encoder
        .positionConversionFactor(Constants.ElevatorPositionConversionFactor)
        .velocityConversionFactor(Constants.ElevatorVelocityConversionFactor);
    rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.ElevatorKP, Constants.ElevatorKI, Constants.ElevatorKD)
        .outputRange(-1, 1);
    rightConfig.closedLoop.maxMotion
        .maxVelocity(Constants.ElevatorMotionMaxVelocity)
        .maxAcceleration(Constants.ElevatorMotionMaxAcceleration)
        .allowedClosedLoopError(Constants.ElevatorMotionAllowedError);
    rightConfig.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    rightConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(Constants.ElevatorSoftLimMax)
        .reverseSoftLimitEnabled(false);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    PIDController = RightMotor.getClosedLoopController();

    if (AtBottom()) {
      zeroEncoder();
    }
    // GoToPosition(getHeight());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elelvator At Bottom", AtBottom());
    SmartDashboard.putBoolean("Elevator At Top", AtTop());
    SmartDashboard.putNumber("Elevator Encoder", getHeight());

    // setpoint = profile.calculate(0.02, setpoint, goal);

    // PIDController.setReference(setpoint.position, ControlType.kPosition,
    // ClosedLoopSlot.kSlot0, FeedForward.calculate(setpoint.velocity));
  }

  public void ElevatorUp() {
    RightMotor.set(Constants.ElevatorSpeed);
  }

  public void ElevatorDown() {
    RightMotor.set(-1 * Constants.ElevatorSpeed);
  }

  public void ElevatorStop() {
    RightMotor.set(0);
  }

  public void SetSoftMax(double newSoftMax) {
    SoftMax = newSoftMax;
  }

  public void GoToPosition(double newTargetPosition) {
    newTargetPosition = Math.min(newTargetPosition, SoftMax);
    newTargetPosition = Math.max(newTargetPosition, Constants.ElevatorSoftLimMin);

    // goal = new TrapezoidProfile.State(newTargetPosition,0);
    PIDController.setReference(newTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0,
        FeedForward.calculate(0));
  }

  public Command GoToPositionCommand(double newSetPoint) {
    return runOnce(() -> {
      GoToPosition(newSetPoint);
    });
  }

  public void zeroEncoder() {
    RightMotor.getEncoder().setPosition(0);
  }

  public Command ZeroEncoderCommand() {
    return runOnce(() -> {
      zeroEncoder();
    });
  }

  public double getHeight() {
    return RightMotor.getEncoder().getPosition();
  }

  public boolean IsLow() {
    return getHeight() < Constants.ElevatorLevel2 - 5.0;
  }

  public boolean AtTop() {
    return RightMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean AtBottom() {
    return RightMotor.getReverseLimitSwitch().isPressed();
  }
}