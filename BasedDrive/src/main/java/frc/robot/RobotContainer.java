// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDefaultCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  //CONTROLLERS
  Joystick m_leftJoystick = new Joystick(Constants.OperatorConstants.LEFT_JOYSTICK);
  Joystick m_rightJoystick = new Joystick(Constants.OperatorConstants.RIGHT_JOYSTICK);
  XboxController m_controller = new XboxController(Constants.OperatorConstants.XBOX_CONTROLLER);

  //SUBSYSTEMS
  SwerveSubsystem m_swerve = new SwerveSubsystem();

  //COMMANDS
  SwerveDefaultCommand m_swerveDefaultCommand = new SwerveDefaultCommand(m_leftJoystick, m_rightJoystick, m_swerve, m_controller);

  //BUTTONS
  private final JoystickButton m_buttonX = new JoystickButton(m_controller, XboxController.Button.kX.value);
  private final JoystickButton m_buttonA = new JoystickButton(m_controller, XboxController.Button.kA.value);

  //Runs in Robot.java, robot init
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //make default command
    m_swerve.setDefaultCommand(m_swerveDefaultCommand);

    //Is this right??
    //I know this is terrible practice but I was a tad curious as to lambda and anonymous class syntax... I'll replace it with something
    //formal if it doesn't work or once a better solution to this problem is found
    //m_buttonX.onTrue(new InstantCommand(() -> m_swerve.zeroYaw()));
    //m_buttonA.onTrue(new InstantCommand(() -> m_swerve.resetAllModuleAbsEncoders()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
