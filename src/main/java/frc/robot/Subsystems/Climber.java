// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkFlex ClimbMotor = new SparkFlex(Constants.ClimberMotorID, MotorType.kBrushless);

  // private DigitalInput HomeSwitch = new
  // DigitalInput(Constants.ClimberHomeSwitch);

  /** Creates a new Climber. */
  public Climber() {
    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.encoder.positionConversionFactor(Constants.ClimberPositionConversionFactor);
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
    ClimbMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ClimberDown() {
    ClimbMotor.set(Constants.ClimberDownSpeed);
  }

  public void ClimberUp() {
    ClimbMotor.set(Constants.ClimberUpSpeed);
  }

  public void ClimberStop() {
    ClimbMotor.set(0);
  }

  public boolean AtHome() {
    return ClimbMotor.getForwardLimitSwitch().isPressed();
  }
}
