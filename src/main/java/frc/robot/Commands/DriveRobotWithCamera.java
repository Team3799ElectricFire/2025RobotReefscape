// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Cameras;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveRobotWithCamera extends Command {
  private final Drivetrain Drivetrain;
  private final Cameras Cams;
  private DoubleSupplier XSupplier, YSupplier, RotSupplier;
  private SlewRateLimiter XLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter YLimiter = new SlewRateLimiter(Constants.panRateOfChangeLimit);
  private SlewRateLimiter RotLimiter = new SlewRateLimiter(Constants.rotRateOfChangeLimit);
  private Rotation2d rotationTarget = null; // Angle to maintain if driver is not trying to turn

  /** Creates a new DriveRobot. */
  public DriveRobotWithCamera(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier rotSupplier) {
    this.Drivetrain = drivetrain;
    this.Cams = drivetrain.eyeballCameras;
    this.XSupplier = xSupplier;
    this.YSupplier = ySupplier;
    this.RotSupplier = rotSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xRawDemand = -1 * XSupplier.getAsDouble();
    double yRawDemand = -1 * YSupplier.getAsDouble();
    double rotRawDemand = -1 * RotSupplier.getAsDouble();

    // magnitude
    double leftMagnatude = Math.sqrt(xRawDemand * xRawDemand + yRawDemand * yRawDemand);
    double rightMagnatude = Math.abs(rotRawDemand);

    // Check if using camera or driver is trying to move
    // TODO track which tag we're looking for based on what camera is active
    boolean isAiming =  false; //targetID != AprilTagIDs.NoTarget;
    boolean isDriving = leftMagnatude > Constants.minThumbstickMagnitude;
    boolean isTurning = rightMagnatude > Constants.minThumbstickMagnitude;
    

     // If driver is not trying to drive set raw demand to zero
     if (!isDriving) {
      xRawDemand = 0;
      yRawDemand = 0;
    }


    // If driver is not trying to turn, prevent rotation with PID
    if (isDriving && !isTurning) {
      // Just stopped turning, rotation target still not set
      if (rotationTarget == null) {
        // Set target to most recent heading
        rotationTarget = Drivetrain.getPose().getRotation();
      }

      // Calculate error between target and current heading
      Rotation2d error = rotationTarget.minus(Drivetrain.getPose().getRotation());

      // Pass turning command to modules
      rotRawDemand = error.getDegrees() * Constants.teleAngleHoldFactor;
    } else {
      // Stopped driving OR started turning, stop trying to hold heading
      rotationTarget = null;
    }


    // If back camera is on, look for coral station AprilTags
    if (isAiming) {
      // Get angle to apriltag camera sees
      Optional<Double> angle = Cams.getAngleToCoralStation();
      
      if (angle.isPresent()) {
        // Overwrite driver turning command if camera found apriltag
        rotRawDemand = -1 * Constants.teleCameraHoldFactor * angle.get();
      }
    }


    if (isTurning && !isDriving  && !isAiming) {
      // sligtly reduce sensitivity if turning in place
      rotRawDemand = rotRawDemand * 0.9;
    }


    double xDemand = XLimiter.calculate(xRawDemand * Math.abs(xRawDemand));
    double yDemand = YLimiter.calculate(yRawDemand * Math.abs(yRawDemand));
    double rotDemand = RotLimiter.calculate(rotRawDemand * Math.abs(rotRawDemand));

    // Only command the modules to move if the driver input is far enough from
    // center
    if (isDriving || isTurning || isAiming) {
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
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
