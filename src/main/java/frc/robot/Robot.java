// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Logged;
import monologue.Monologue;


public class Robot extends TimedRobot implements Logged {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();
    Monologue.setupMonologue(this, "Robot", false, false);
  }
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
     // setFileOnly is used to shut off NetworkTables broadcasting for most logging calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    Monologue.setFileOnly(DriverStation.isFMSAttached());
    // This method needs to be called periodically, or no logging annotations will
    // process properly.
    Monologue.updateAll();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      m_robotContainer.setAlliance(ally.get());
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      m_robotContainer.setAlliance(ally.get());
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
