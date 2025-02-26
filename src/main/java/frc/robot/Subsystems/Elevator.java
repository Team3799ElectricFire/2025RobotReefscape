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
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkFlex LeftMotor = new SparkFlex(Constants.ElevatorLeftMotorID, MotorType.kBrushless);
  private SparkFlex RightMotor = new SparkFlex(Constants.ElevatorRightMotorID, MotorType.kBrushless);
  private ElevatorFeedforward FeedForward = new ElevatorFeedforward(Constants.ElevatorKS, Constants.ElevatorKG, Constants.ElevatorKV);
  private SparkClosedLoopController PIDController;
  private double SoftMax = Constants.ElevatorSoftLimMax;

  /** Creates a new Elevator. */
  public Elevator() {
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.follow(Constants.ElevatorRightMotorID, true);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.encoder.positionConversionFactor(Constants.ElevatorPositionConversionFactor);
    rightConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.ElevatorKP, Constants.ElevatorKI, Constants.ElevatorKD)
        .outputRange(-1, 1);
    rightConfig.closedLoop.maxMotion
        .maxVelocity(Constants.ElevatorMotionMaxVelocity)
        .maxAcceleration(Constants.ElevatorMotionMaxAcceleration)
        .allowedClosedLoopError(Constants.ElevatorMotionAllowedError);
    rightConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed)
        .reverseLimitSwitchEnabled(true)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    PIDController = RightMotor.getClosedLoopController();

    if (AtBottom()) {
      zeroEncoder();
    }
    //GoToPosition(getHeight());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public void SetSoftMax(double newSoftMax){
    SoftMax = newSoftMax;
  }
  
  public void GoToPosition(double newTargetPosition) {
    newTargetPosition = Math.min(newTargetPosition, SoftMax);
    newTargetPosition = Math.max(newTargetPosition, Constants.ElevatorSoftLimMin);
    PIDController.setReference(newTargetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, FeedForward.calculate(0));
  }

  public Command GoToPositionCommand(double newSetPoint){
    return run(() -> {GoToPosition(newSetPoint);}); 
  }

  public void zeroEncoder (){
    RightMotor.getEncoder().setPosition(0);
  }

  public double getHeight(){
    return RightMotor.getEncoder().getPosition();
  }

  public boolean IsLow() {
    return getHeight() < Constants.ElevatorLevel2 - 5.0; 
  }

  public boolean AtTop(){
    return RightMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean AtBottom(){
    return RightMotor.getReverseLimitSwitch().isPressed();
  }
}