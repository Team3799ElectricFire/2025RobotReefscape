// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristHome extends Command {
  private final Wrist Wrost;
  /** Creates a new WristHome. */
  public WristHome(Wrist wrist) {
    Wrost = wrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Wrost.WristUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Wrost.WristStop();
    Wrost.HomeEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Wrost.AtHome();
  }
}
