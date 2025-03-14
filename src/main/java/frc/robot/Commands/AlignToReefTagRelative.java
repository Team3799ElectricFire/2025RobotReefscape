// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer abortTimer, settleTimer;
  private Drivetrain drivebase;
  private int tagID = -1;
  private Pose2d goalPose;
  private final StructPublisher<Pose2d> goalPublisher, tagPublisher;

  public AlignToReefTagRelative(Drivetrain drivebase, boolean isRightScore) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0); // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0); // Horitontal movement
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0); // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);

    goalPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/GoalPose", Pose2d.struct).publish();
    tagPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/TagPose", Pose2d.struct).publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.settleTimer = new Timer();
    this.settleTimer.start();
    this.abortTimer = new Timer();
    this.abortTimer.start();

    var targetTag = drivebase.Cams.getLowCameraTag();
    // End this command instantly if no tag was seen
    if (targetTag.isPresent()) {
      tagID = targetTag.get().getFiducialId();

      SmartDashboard.putNumber("TAG SEEN", tagID);

      var targetPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID);

      // End this command instantly if tag seen is not part of field
      if (targetPose.isPresent()) {
        Pose2d tagPose = targetPose.get().toPose2d();

        // Publish tagPose for debug
        tagPublisher.set(tagPose);

        Pose2d offset = new Pose2d(
            new Translation2d(Constants.X_SETPOINT_REEF_ALIGNMENT,
                isRightScore ? -Constants.Y_SETPOINT_REEF_ALIGNMENT : Constants.Y_SETPOINT_REEF_ALIGNMENT),
            new Rotation2d());

        // Goal pose we want robot to reach, relative to AprilTag
        goalPose = tagPose.plus(new Transform2d( // Tag Pose2d of AprilTag and add...
            offset.rotateBy(tagPose.getRotation()).getTranslation(), // ... desired offset, rotated by angle of AprilTag...
            Rotation2d.fromDegrees(180))); // ... and facing the opposite direction, back towards AprilTag

        // Publish goalPose for debug
        goalPublisher.set(goalPose);

        // Set PID controller setpoints
        rotController.setSetpoint(goalPose.getRotation().getDegrees());
        rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(goalPose.getX());
        xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(goalPose.getY());
        yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drivebase.getPose();

    double xSpeed = xController.calculate(robotPose.getX());
    double ySpeed = yController.calculate(robotPose.getY());
    double rotSpeed = rotController.calculate(robotPose.getRotation().getDegrees());

    drivebase.driveFieldRelative(xSpeed, ySpeed, rotSpeed);

    if (!xController.atSetpoint() || !yController.atSetpoint() || !rotController.atSetpoint()) {
      settleTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.abortTimer.hasElapsed(Constants.REEF_ALIGN_MAX_TIME) ||
        settleTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
