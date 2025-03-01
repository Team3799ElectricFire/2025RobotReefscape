// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
  private SparkMax LeftMotor = new SparkMax(Constants.CoralLeftMotorID, MotorType.kBrushless);
  private SparkMax RightMotor = new SparkMax(Constants.CoralRightMotorID, MotorType.kBrushless);

  private SparkLimitSwitch TopSwitch = LeftMotor.getForwardLimitSwitch();
  private SparkLimitSwitch BottomSwitch = LeftMotor.getReverseLimitSwitch();

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.limitSwitch
        .forwardLimitSwitchEnabled(false)
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .reverseLimitSwitchEnabled(false)
        .reverseLimitSwitchType(Type.kNormallyClosed);
    leftConfig.follow(Constants.CoralRightMotorID, true);
    LeftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.inverted(true);
    RightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printDS();
  }

  private void printDS() {
    SmartDashboard.putBoolean("HaveCoral", HaveCoral());
    SmartDashboard.putBoolean("IsSafe", IsSafeCoral());
  }

  public void CoralForward() {
    RightMotor.set(Constants.CoralIntakeSpeed);
  }

  public void CoralPeg() {
    RightMotor.set(Constants.CoralOutakeSpeed);
  }
  
  public void CoralShelf() {
    LeftMotor.pauseFollowerMode();
    RightMotor.set(Constants.CoralOutakeSpeed);
    LeftMotor.set(Constants.CoralShelfSpeed);
  }

  public void CoralBackward() {
    RightMotor.set(Constants.CoralSecureSpeed);
  }

  public void CoralStop() {
    LeftMotor.resumeFollowerMode();
    RightMotor.set(0);
  }

  public boolean HaveCoral() {
    return !BottomSwitch.isPressed();
  }

  public boolean IsSafeCoral() {
    return TopSwitch.isPressed();
  }

}
