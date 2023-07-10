// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDefaultCommand extends CommandBase {
    /** Creates a new Command. */

    SwerveSubsystem m_swerveSubsystem;
    Joystick m_leftJoystick;
    Joystick m_rightJoystick;

    public SwerveDefaultCommand(Joystick p_rightJoystick, Joystick p_leftJoystick, SwerveSubsystem p_swerveSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_swerveSubsystem = p_swerveSubsystem;
        m_leftJoystick = p_leftJoystick;
        m_rightJoystick = p_rightJoystick;
        addRequirements(m_swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
        @Override
    public void execute() {
        m_swerveSubsystem.Swerve(m_leftJoystick.getY(), m_leftJoystick.getX(), m_rightJoystick.getX());
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
