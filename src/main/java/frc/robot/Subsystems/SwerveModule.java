package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private SparkFlex SteerMotor, DriveMotor;
    private SparkFlexConfig SteerConfig, DriveConfig;
    private AbsoluteEncoder SteerEncoder;
    private RelativeEncoder DriveEncoder;
    private SparkClosedLoopController DrivePID, SteerPID;
    private double SteerOffset;
    private SwerveModuleState DesiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveMotorID, int steerMotorID, double steerOffset) {
        // Motors
        SteerMotor = new SparkFlex(steerMotorID, MotorType.kBrushless);
        DriveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);

        // encoders
        SteerEncoder = SteerMotor.getAbsoluteEncoder();
        DriveEncoder = DriveMotor.getEncoder();

        // PIDs
        SteerPID = SteerMotor.getClosedLoopController();
        DrivePID = DriveMotor.getClosedLoopController();

        this.SteerOffset = steerOffset;

        // Motor Configuration
        SteerConfig = new SparkFlexConfig();
        SteerConfig.encoder.positionConversionFactor(Constants.SteerMotorPositionFactor);
        SteerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(Constants.SteeringPgain)
                .i(Constants.SteeringIgain)
                .d(Constants.SteeringDgain)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .outputRange(-1, 1);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DriveConfig = new SparkFlexConfig();
        DriveConfig.encoder.velocityConversionFactor(Constants.DriveMotorVelocityFactor);
        DriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.DrivingPgain)
                .i(Constants.DrivingIgain)
                .d(Constants.DrivingDgain)
                .outputRange(-1, 1);
        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                DriveEncoder.getVelocity(),
                new Rotation2d(SteerEncoder.getPosition() - SteerOffset));
    }

    public SwerveModuleState getDesiredPosition() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                DriveEncoder.getPosition(),
                new Rotation2d(SteerEncoder.getPosition() - SteerOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Correct incoming desired state (which is relative to robot) with this modules
        // offset angle to get a state relative to this modules mounting
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(SteerOffset));

        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
                correctedDesiredState,
                new Rotation2d(SteerEncoder.getPosition()));

        // Always set the driving motor's speed
        DrivePID.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

        // But only set the steering motor's position if the driving motor is moving
        if (Math.abs(optimizedDesiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) {
            SteerPID.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);
        }

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void setDesiredStateNoRestrictions(SwerveModuleState desiredState) {
        // Correct incoming desired state (which is relative to robot) with this modules
        // offset angle to get a state relative to this modules mounting
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(SteerOffset));

        // Optimize desired state based on current angle (never rotate module more than
        // 90degrees)
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
                correctedDesiredState,
                new Rotation2d(SteerEncoder.getPosition()));

        // Always set the driving motor's speed
        DrivePID.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);

        // Always set the steering motor's position
        SteerPID.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void stop() {
        DriveMotor.set(0.0);
        SteerMotor.set(0.0);
    }

    public void resetEncoders() {
        DriveEncoder.setPosition(0);
    }

}
