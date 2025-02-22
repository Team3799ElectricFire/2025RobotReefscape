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

        // Motor Configuration
        SteerConfig = new SparkFlexConfig();
        SteerConfig.inverted(true);
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
        SteerConfig.absoluteEncoder
                .positionConversionFactor(Constants.SteerMotorPositionFactor)
                .zeroOffset(steerOffset);
        SteerMotor.configure(SteerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        DriveConfig = new SparkFlexConfig();
        DriveConfig.inverted(true);
        DriveConfig.encoder.velocityConversionFactor(Constants.DriveMotorVelocityFactor);
        DriveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(Constants.DrivingPgain)
                .i(Constants.DrivingIgain)
                .d(Constants.DrivingDgain)
                .velocityFF(Constants.DrivingFFgain)
                .outputRange(-1, 1);
        DriveMotor.configure(DriveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                DriveEncoder.getVelocity(),
                new Rotation2d(SteerEncoder.getPosition()));
    }

    public SwerveModuleState getDesiredPosition() {
        return DesiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                DriveEncoder.getPosition(),
                new Rotation2d(SteerEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {       
        // Optimize desired state based on current angle (never rotate module more than 90degrees)
        desiredState.optimize(new Rotation2d(SteerEncoder.getPosition()));

        // Always set the driving motor's speed
        DrivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        // But only set the steering motor's position if the driving motor is moving
        if (Math.abs(desiredState.speedMetersPerSecond) > Constants.kMinSpeedMetersPerSecond) {
            SteerPID.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
        }

        // Update last recorded desired state
        this.DesiredState = desiredState;
    }

    public void setDesiredStateNoRestrictions(SwerveModuleState desiredState) {   
        // Optimize desired state based on current angle (never rotate module more than 90degrees)
        desiredState.optimize(new Rotation2d(SteerEncoder.getPosition()));

        // Always set the driving motor's speed
        DrivePID.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);

        // Always set the steering motor's position
        SteerPID.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

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
