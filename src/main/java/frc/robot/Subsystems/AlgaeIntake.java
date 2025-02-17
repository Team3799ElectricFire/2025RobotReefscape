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
  private SparkMax AlgaeMotor = new SparkMax(Constants.AlgaeMotorID, MotorType.kBrushless);

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    AlgaeMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void AlgaeTakeIn() {
    AlgaeMotor.set(Constants.AlgaeIntakeSpeed);
  }

  public void AlgaeTakeOut() {
    AlgaeMotor.set(-1 * Constants.AlgaeOutakeSpeed);
  }

  public void AlgaeHold() {
    AlgaeMotor.set(Constants.AlgaeHold);
  }

  public void AlgaeStop() {
    AlgaeMotor.set(0);
  }
}
