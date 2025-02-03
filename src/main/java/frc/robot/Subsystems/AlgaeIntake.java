// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private SparkMax LeftMotor = new SparkMax(Constants.AlgaeLeftMotorID, MotorType.kBrushless);
  private SparkMax RightMotor = new SparkMax(Constants.AlgaeRightMotorID, MotorType.kBrushless);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(true);
    leftConfig.follow(Constants.AlgaeRightMotorID);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void AlgaeTakeIn() {
    RightMotor.set(Constants.AlgaeIntakeSpeed);

  }
  public void AlgaeTakeOut() {
    RightMotor.set(-1 * Constants.AlgaeOutakeSpeed);

  }
  public void AlgaeHold() {
    RightMotor.set(Constants.AlgaeHold);
  }
  public void AlgaeStop (){
    RightMotor.set(0);
  }
}
