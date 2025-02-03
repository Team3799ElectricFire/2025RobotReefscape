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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private SparkFlex LeftMotor = new SparkFlex(Constants.ClimberLeftMotorID, MotorType.kBrushless);
  private SparkFlex RightMotor = new SparkFlex(Constants.ClimberRightMotorID, MotorType.kBrushless);

  // private DigitalInput HomeSwitch = new
  // DigitalInput(Constants.ClimberHomeSwitch);

  /** Creates a new Climber. */
  public Climber() {
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.inverted(true);
    leftConfig.follow(Constants.ClimberRightMotorID);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.encoder.positionConversionFactor(Constants.ClimberPositionConversionFactor);
    rightConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ClimberDown() {

    RightMotor.set(Constants.ClimberDownSpeed);

  }

  public void ClimberUp() {

    RightMotor.set(Constants.ClimberUpSpeed);

  }

  public void ClimberStop() {

    RightMotor.set(0);

  }

  public boolean AtHome() {

    return RightMotor.getForwardLimitSwitch().isPressed();

  }

}
