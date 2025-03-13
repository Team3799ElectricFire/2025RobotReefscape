// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drivetrain drivebase;
  private int tagID = -1;

  public AlignToReefTagRelative(Drivetrain drivebase, boolean isRightScore) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    var targetTag = drivebase.Cams.getLowCameraTag();
    // End this command instantly if no tag was seen
    if (targetTag.isEmpty()) this.end(false);

    tagID = targetTag.get().getFiducialId();
    var targetPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID);

    // End this command instantly if tag seen is not part of field
    if (targetPose.isEmpty()) this.end(false);

    // rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    // rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    // xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    // xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    // yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
    // yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetTag = drivebase.Cams.getLowCameraTag();
    targetTag.ifPresent(
      tag -> {
        if (tag.getFiducialId() == tagID) {
          this.dontSeeTagTimer.reset();

          

        }
      }
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
      stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
