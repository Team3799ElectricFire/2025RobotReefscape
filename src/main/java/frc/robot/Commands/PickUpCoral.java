// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.CoralIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PickUpCoral extends Command {
  private final CoralIntake Intake;

  /** Creates a new PickUpCoral. */
  public PickUpCoral(CoralIntake coralintake) {
    Intake = coralintake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.CoralSend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.CoralStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Intake.IsSafeCoral() && Intake.HaveCoral();
  }
}
