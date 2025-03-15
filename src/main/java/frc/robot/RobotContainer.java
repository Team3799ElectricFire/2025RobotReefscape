// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {
  private Drivetrain Drivetrain = new Drivetrain();
  private Cameras Cams = Drivetrain.Cams;
  private CoralIntake CoralIntake = new CoralIntake();
  private Climber Climber = new Climber();
  private AlgaeIntake Algae = new AlgaeIntake();
  private Elevator Elevate = new Elevator();
  private Wrist Wrost = new Wrist();

  private CommandXboxController Driver = new CommandXboxController(0);
  private CommandXboxController Copilot = new CommandXboxController(1);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // named comands for pathplanner
    NamedCommands.registerCommand("CoralFromStation", new PickUpCoral(CoralIntake));
    NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(CoralIntake).withTimeout(0.5));
    NamedCommands.registerCommand("ScoreCoralLow", new ScoreCoralLow(CoralIntake).withTimeout(1));
    NamedCommands.registerCommand("PickUpAlgae", new PickUpAlgae(Algae).withTimeout(1));
    NamedCommands.registerCommand("ScoreAlgae", new ScoreAlgae(Algae).withTimeout(0.5));
    NamedCommands.registerCommand("ElevatorLevel1", Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
    NamedCommands.registerCommand("ElevatorLevel2", Elevate.GoToPositionCommand(Constants.ElevatorLevel2));
    NamedCommands.registerCommand("ElevatorLevel3", Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
    NamedCommands.registerCommand("ElevatorLevel4", Elevate.GoToPositionCommand(Constants.ElevatorLevel4));
    NamedCommands.registerCommand("WristFloorPickup", Wrost.GoToPositionCommand(Constants.WristFloorPickUp));
    NamedCommands.registerCommand("WristReefPickup", Wrost.GoToPositionCommand(Constants.WristReefPickUp));
    NamedCommands.registerCommand("WristHome", Wrost.GoToPositionCommand(Constants.WristStart));
    NamedCommands.registerCommand("WristTravel", Wrost.GoToPositionCommand(Constants.WristTravel));
    NamedCommands.registerCommand("WristScore", Wrost.GoToPositionCommand(Constants.WristScore));

    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setAlliance(Alliance color) {
    Cams.setAlliance(color);
    Drivetrain.setAlliance(color);
  }

  private void configureBindings() {
    // Drivetrain
    Drivetrain.setDefaultCommand(new DriveRobotWithCamera(Drivetrain, Driver::getLeftY, Driver::getLeftX, Driver::getRightX));
    Driver.start().onTrue(Drivetrain.ZeroHeadingCommand());
    Driver.back().onTrue(Drivetrain.toggleDriveRobotRelativeCommand());
    Driver.leftStick().onTrue(Drivetrain.setLowSpeedCommand());
    Driver.rightStick().onTrue(Drivetrain.setHgihSpeedCommand());

    // Coral
    Driver.leftBumper().onTrue(new SequentialCommandGroup(
        Elevate.LockElevatorCommand(),
        new PickUpCoral(CoralIntake).andThen(new SecureCoral(CoralIntake)),
        Elevate.UnLockElevatorCommand()).onlyIf(Elevate::IsLow));
    Driver.leftTrigger().onTrue(new ConditionalCommand(
        new ScoreCoralLow(CoralIntake),
        new ScoreCoral(CoralIntake),
        Elevate::IsLow));

    // Algae
    SmartDashboard.putData("Wrist UP Command", new WristUp(Wrost));
    SmartDashboard.putData("Wrist DOWN Command", new WristDown(Wrost));
    SmartDashboard.putData("Home Wrist Command", new WristHome(Wrost));
    SmartDashboard.putData("Reset Wrist Encoder", Wrost.HomeEncoderCommand());
    Driver.rightBumper().whileTrue(new SequentialCommandGroup(
        new ConditionalCommand(
            Wrost.GoToPositionCommand(Constants.WristFloorPickUp),
            Wrost.GoToPositionCommand(Constants.WristReefPickUp),
            Elevate::IsLow),
        new PickUpAlgae(Algae)));
    Driver.rightBumper().onFalse(new SequentialCommandGroup(
      Wrost.GoToPositionCommand(Constants.WristTravel)));
    Driver.rightTrigger().whileTrue(new SequentialCommandGroup(
        Wrost.GoToPositionCommand(Constants.WristScore),
        new ScoreAlgae(Algae)));
    Driver.rightTrigger().onFalse(Wrost.GoToPositionCommand(Constants.WristStart));

    Copilot.rightBumper().whileTrue(new SequentialCommandGroup(
        new ConditionalCommand(
            Wrost.GoToPositionCommand(Constants.WristFloorPickUp),
            Wrost.GoToPositionCommand(Constants.WristReefPickUp),
            Elevate::IsLow),
        new PickUpAlgae(Algae)));
    Copilot.rightBumper().onFalse(new SequentialCommandGroup(
      Wrost.GoToPositionCommand(Constants.WristTravel)));
    Copilot.rightTrigger().whileTrue(new SequentialCommandGroup(
        Wrost.GoToPositionCommand(Constants.WristScore),
        new ScoreAlgae(Algae)));
    Copilot.rightTrigger().onFalse(Wrost.GoToPositionCommand(Constants.WristStart));

    // Climber
    Driver.povDown().whileTrue(new ClimberDown(Climber));
    Driver.povUp().whileTrue(new ClimberUp(Climber));

    // Elevator
    SmartDashboard.putData("Elevator UP Command", new ElevatorUp(Elevate));
    SmartDashboard.putData("Elevator DOWN Command", new ElevatorDown(Elevate));
    SmartDashboard.putData("Home Elevator Command", new ElevatorHome(Elevate));
    SmartDashboard.putData("Reset Elevator Encoder", Elevate.ZeroEncoderCommand());
    Driver.a().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel1).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.HighSpeedMultiple)
    ));
    Driver.x().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel2).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L2SpeedMultiple)
    ));
    Driver.b().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel3).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L3SpeedMultiple)
    ));
    Driver.y().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel4).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L4SpeedMultiple)
    ));

    Copilot.povDown().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel1).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.HighSpeedMultiple)
    ));
    Copilot.povLeft().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel2).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L2SpeedMultiple)
    ));
    Copilot.povRight().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel3).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L3SpeedMultiple)
    ));
    Copilot.povUp().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel4).alongWith(
      Drivetrain.setDriveSpeeCommand(Constants.L4SpeedMultiple)
    ));

    // Cameras
    Copilot.a().onTrue(Drivetrain.TurnOnHighFCameraCommand()); // A == Processor (score algae)
    Copilot.a().onFalse(Drivetrain.TurnOffHighFCameraCommand());
    Copilot.y().onTrue(Drivetrain.TurnOnLowCameraCommand()); // Y == Reef (score coral)
    Copilot.y().onFalse(Drivetrain.TurnOffLowCameraCommand());
    Copilot.b().onTrue(Drivetrain.TurnOnBackCameraCommand()); // B == Coral Station (pickup coral)
    Copilot.b().onFalse(Drivetrain.TurnOffBackCameraCommand());

    // Driver Assist
    //Copilot.rightTrigger().whileTrue(new AlignToReefTagRelative(Drivetrain, true));
    //Copilot.leftTrigger().whileTrue(new AlignToReefTagRelative(Drivetrain, false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
