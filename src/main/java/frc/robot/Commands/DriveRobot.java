// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveRobot extends Command {
  private final Drivetrain Drivetrain;
  private DoubleSupplier XSupplier, YSupplier, RotSupplier;
  private SlewRateLimiter XLimiter = new SlewRateLimiter(8.0);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(8.0);
  private SlewRateLimiter RotLimiter = new SlewRateLimiter(8.0);

  /** Creates a new DriveRobot. */
  public DriveRobot(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotSupplier) {
    this.Drivetrain = drivetrain;
    this.XSupplier = xSupplier;
    this.YSupplier = ySupplier;
    this.RotSupplier = rotSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xRawDemand = -1 * XSupplier.getAsDouble(); 
    double yRawDemand = -1 *YSupplier.getAsDouble();
    double rotRawDemand = -1 * RotSupplier.getAsDouble();

    double xDemand = XLimiter.calculate( xRawDemand * Math.abs(xRawDemand));
    double yDemand = YLimiter.calculate( yRawDemand * Math.abs(yRawDemand));
    double rotDemand = RotLimiter.calculate( rotRawDemand * Math.abs(rotRawDemand));

    //magnitude
     double leftMagnatude = Math.sqrt(xRawDemand*xRawDemand + yRawDemand*yRawDemand);
     double rightMagnatude = Math.abs(rotRawDemand);

    // Only command the modules to move if the driver input is far enough from center
    if (leftMagnatude > Constants.minThumbstickMagnitude || rightMagnatude > Constants.minThumbstickMagnitude) {
      // Drive
      if (Drivetrain.getDriveRobotRelative()) {
        Drivetrain.driveRobotRelative(xDemand, yDemand, rotDemand);
  
      } else {
        Drivetrain.driveFieldRelative(xDemand, yDemand, rotDemand);
  
      }
    } else {
      // Stop
      Drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
