// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

    //CONTROLLERS
    Joystick m_leftJoystick = new Joystick(Constants.OperatorConstants.LEFT_JOYSTICK);
    Joystick m_rightJoystick = new Joystick(Constants.OperatorConstants.RIGHT_JOYSTICK);

    //SUBSYSTEMS
    SwerveSubsystem m_swerve = new SwerveSubsystem();

    //COMMANDS
    SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_leftJoystick, m_rightJoystick, m_swerve);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
