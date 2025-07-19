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
  //private Climber Claimber = new Climber();
  private AlgaeIntake Algae = new AlgaeIntake();
  private Elevator Elevate = new Elevator();
  private Wrist Wrost = new Wrist();

  private CommandXboxController Driver = new CommandXboxController(0);
  //private CommandXboxController Copilot = new CommandXboxController(1);
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
    //SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setAlliance(Alliance color) {
    Cams.setAlliance(color);
    Drivetrain.setAlliance(color);
  }

  private void configureBindings() {
    // Drivetrain
    Drivetrain.setDefaultCommand(new DriveRobot(Drivetrain, Driver::getLeftY, Driver::getLeftX, Driver::getRightX));
    Driver.start().onTrue(Drivetrain.ZeroHeadingCommand());
    Driver.back().onTrue(Drivetrain.toggleDriveRobotRelativeCommand());

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

    // Climber

    // Elevator
    SmartDashboard.putData("Elevator UP Command", new ElevatorUp(Elevate));
    SmartDashboard.putData("Elevator DOWN Command", new ElevatorDown(Elevate));
    SmartDashboard.putData("Home Elevator Command", new ElevatorHome(Elevate));
    SmartDashboard.putData("Reset Elevator Encoder", Elevate.ZeroEncoderCommand());
    Driver.a().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel1));
    Driver.x().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel2));
    Driver.b().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel3));
    Driver.y().onTrue(Elevate.GoToPositionCommand(Constants.ElevatorLevel4));

    // Cameras

    // Driver Assist
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
