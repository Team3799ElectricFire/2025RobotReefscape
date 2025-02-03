// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private SparkMax LeftMotor = new SparkMax(Constants.CoralLeftMotorID, MotorType.kBrushless);
  private SparkMax RightMotor = new SparkMax(Constants.CoralRightMotorID, MotorType.kBrushless);

  private DigitalInput TopSwitch = new DigitalInput(Constants.CoralTopSwitch);
  private DigitalInput BottomSwitch = new DigitalInput(Constants.CoralBottomSwitch);

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.inverted(true);
    leftConfig.follow(Constants.CoralRightMotorID);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public void CoralSend() {
    RightMotor.set(Constants.CoralIntakeSpeed);
  }

  public void CoralShelf(){
    RightMotor.set(Constants.CoralOutakeSpeed);
    LeftMotor.set(Constants.CoralShelfSpeed);
  }
  
  public void CoralStop() {
    RightMotor.set(0);
  }
  public boolean HaveCoral(){
    return BottomSwitch.get();
  }
  public boolean IsSafeCoral() {
    return !TopSwitch.get();
  }
  
}
