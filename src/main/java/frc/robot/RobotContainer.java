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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {
  private Drivetrain Drivetrain = new Drivetrain();
  private CoralIntake CoralIntake = new CoralIntake();
  private Climber Climber = new Climber();
  private AlgaeIntake Algae = new AlgaeIntake();
  private Elevator Elevate = new Elevator();
  private Wrist Wrost = new Wrist();
  private Cameras Cams = new Cameras();
  private CommandXboxController Gamepad = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // named comands for pathplanner
    NamedCommands.registerCommand("CoralFromStation", new PickUpCoral(CoralIntake));
    NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(CoralIntake));
    NamedCommands.registerCommand("ScoreCoralLow", new ScoreCoralLow(CoralIntake));
    NamedCommands.registerCommand("PickUpAlgae",new PickUpAlgae(Algae));
    NamedCommands.registerCommand("ScoreAlgae",new ScoreAlgae(Algae));
    NamedCommands.registerCommand("ElevatorLevel1", Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
    NamedCommands.registerCommand("ElevatorLevel2", Elevate.GoToPositionCommand(Constants.ElevatorLevel2));
    NamedCommands.registerCommand("ElevatorLevel3", Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
    NamedCommands.registerCommand("ElevatorLevel4", Elevate.GoToPositionCommand(Constants.ElevatorLevel4));
    NamedCommands.registerCommand("WristPickup", Wrost.GoToPositionCommand(Constants.WristPickUp));
    NamedCommands.registerCommand("WristHome",Wrost.GoToPositionCommand(Constants.WristStart));
    NamedCommands.registerCommand("WristTravel", Wrost.GoToPositionCommand(Constants.WristTravel));
    NamedCommands.registerCommand("WristScore", Wrost.GoToPositionCommand(Constants.WristScore));
    
    configureBindings();
  }

  public void setAlliance(Alliance color) {
    Cams.setAlliance(color);
  }

  private void configureBindings() {
    Drivetrain.setDefaultCommand(new DriveRobot(Drivetrain, Gamepad::getLeftY, Gamepad::getLeftX, Gamepad::getRightX));
    Gamepad.leftBumper().whileTrue(new PickUpCoral(CoralIntake));
    Gamepad.leftBumper().onFalse(new ConditionalCommand(
      new InstantCommand(() -> {Elevate.SetSoftMax(Constants.ElevatorSoftLimMax);}),
      new InstantCommand(() -> {Elevate.SetSoftMax(Constants.ElevatorSoftLimCoral);}),
      CoralIntake::IsSafeCoral));


    Gamepad.povDown().whileTrue(new ClimberDown(Climber));
    Gamepad.povUp().whileTrue(new ClimberUp(Climber));
    Gamepad.rightBumper().whileTrue(new PickUpAlgae(Algae));
    Gamepad.rightTrigger().whileTrue(new ScoreAlgae(Algae));

    Gamepad.a().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
    Gamepad.b().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
    Gamepad.x().onTrue(Wrost.GoToPositionCommand(Constants.WristPickUp));
    Gamepad.y().onTrue(Wrost.GoToPositionCommand(Constants.WristStart));

    Gamepad.leftTrigger().whileTrue(new ConditionalCommand(
      new ScoreCoralLow(CoralIntake), 
      new ScoreCoral(CoralIntake), 
      Elevate::IsLow));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
