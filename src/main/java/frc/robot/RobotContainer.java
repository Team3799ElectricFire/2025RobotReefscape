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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {
  private Drivetrain Drivetrain = new Drivetrain();
  private Cameras Cams = Drivetrain.eyeballCameras;
  private CoralIntake CoralIntake = new CoralIntake();
  private Climber Climber = new Climber();
  private AlgaeIntake Algae = new AlgaeIntake();
  private Elevator Elevate = new Elevator();
  private Wrist Wrost = new Wrist();
  
  private CommandXboxController Gamepad = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // named comands for pathplanner
    /*
     * NamedCommands.registerCommand("CoralFromStation", new
     * PickUpCoral(CoralIntake));
     * NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(CoralIntake));
     * NamedCommands.registerCommand("ScoreCoralLow", new
     * ScoreCoralLow(CoralIntake));
     * NamedCommands.registerCommand("PickUpAlgae",new PickUpAlgae(Algae));
     * NamedCommands.registerCommand("ScoreAlgae",new ScoreAlgae(Algae));
     * NamedCommands.registerCommand("ElevatorLevel1",
     * Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
     * NamedCommands.registerCommand("ElevatorLevel2",
     * Elevate.GoToPositionCommand(Constants.ElevatorLevel2));
     * NamedCommands.registerCommand("ElevatorLevel3",
     * Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
     * NamedCommands.registerCommand("ElevatorLevel4",
     * Elevate.GoToPositionCommand(Constants.ElevatorLevel4));
     * NamedCommands.registerCommand("WristPickup",
     * Wrost.GoToPositionCommand(Constants.WristPickUp));
     * NamedCommands.registerCommand("WristHome",Wrost.GoToPositionCommand(Constants
     * .WristStart));
     * NamedCommands.registerCommand("WristTravel",
     * Wrost.GoToPositionCommand(Constants.WristTravel));
     * NamedCommands.registerCommand("WristScore",
     * Wrost.GoToPositionCommand(Constants.WristScore));
     */

    configureBindings();
  }

  public void setAlliance(Alliance color) {
    Cams.setAlliance(color);
    //Cams.setLowDriverMode(false);
    //Cams.setHighBDriverMode(false);
    //Cams.setHighFDriverMode(false);
  }

  private void configureBindings() {
    // Drivetrain
    Drivetrain.setDefaultCommand(new DriveRobotWithCamera(Drivetrain, Gamepad::getLeftY, Gamepad::getLeftX, Gamepad::getRightX));
    Gamepad.start().onTrue(Drivetrain.ZeroHeadingCommand());
    Gamepad.back().onTrue(new InstantCommand(() -> {Drivetrain.toggleDriveRobotRelative();}));

    // Coral
    Gamepad.leftBumper().whileTrue(new SequentialCommandGroup(
        new InstantCommand(() -> {Drivetrain.IsAimingBackCamera = true;}),
        new PickUpCoral(CoralIntake).andThen(new SecureCoral(CoralIntake))));
    /*Gamepad.leftBumper().whileTrue(new SequentialCommandGroup(
        Drivetrain.TurnOnBackCameraCommand(),
        new PickUpCoral(CoralIntake).andThen(new SecureCoral(CoralIntake))));*/
    Gamepad.leftBumper().onFalse(new SequentialCommandGroup(
        new InstantCommand(() -> {Drivetrain.IsAimingBackCamera = false;}),
        new PickUpCoral(CoralIntake).withTimeout(0.5)
        .andThen(new SecureCoral(CoralIntake)).withTimeout(0.5)));
    /*Gamepad.leftBumper().onFalse(new SequentialCommandGroup(
        Drivetrain.TurnOffBackCameraCommand(),
        new PickUpCoral(CoralIntake).withTimeout(0.5)
        .andThen(new SecureCoral(CoralIntake)).withTimeout(0.5)));*/
    Gamepad.leftTrigger().whileTrue(new ConditionalCommand(
        new ScoreCoralLow(CoralIntake),
        new ScoreCoral(CoralIntake),
        Elevate::IsLow));

    // Algae
    SmartDashboard.putData("Wrist UP Command",new WristUp(Wrost));
    SmartDashboard.putData("Wrist DOWN Command",new WristDown(Wrost));
    SmartDashboard.putData("Home Wrist Command", new WristHome(Wrost));
    SmartDashboard.putData("Reset Wrist Encoder", Wrost.HomeEncoderCommand());
    Gamepad.rightBumper().whileTrue(new SequentialCommandGroup(
        Wrost.GoToPositionCommand(Constants.WristPickUp),
        new PickUpAlgae(Algae)));
    Gamepad.rightBumper().onFalse(Wrost.GoToPositionCommand(Constants.WristTravel));
    Gamepad.rightTrigger().whileTrue(new SequentialCommandGroup(
        Wrost.GoToPositionCommand(Constants.WristScore),
        new ScoreAlgae(Algae)));
    Gamepad.rightTrigger().onFalse(Wrost.GoToPositionCommand(Constants.WristStart));

    // Climber
    Gamepad.povDown().whileTrue(new ClimberDown(Climber));
    Gamepad.povUp().whileTrue(new ClimberUp(Climber));

    // Elevator
    SmartDashboard.putData("Elevator UP Command",new ElevatorUp(Elevate));
    SmartDashboard.putData("Elevator DOWN Command",new ElevatorDown(Elevate));
    SmartDashboard.putData("Home Elevator Command", new ElevatorHome(Elevate));
    SmartDashboard.putData("Reset Elevator Encoder", Elevate.ZeroEncoderCommand());
    Gamepad.a().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
    Gamepad.b().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
    Gamepad.x().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel2));
    Gamepad.y().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel4));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
