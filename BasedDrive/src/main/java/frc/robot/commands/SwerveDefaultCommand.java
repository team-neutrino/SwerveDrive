// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDefaultCommand extends CommandBase {
    /** Creates a new Command. */

    SwerveSubsystem m_swerveSubsystem;
    Joystick m_leftJoystick;
    Joystick m_rightJoystick;
    XboxController m_controller;
    double cycle = 0;

    public SwerveDefaultCommand(Joystick p_rightJoystick, Joystick p_leftJoystick, SwerveSubsystem p_swerveSubsystem, XboxController p_controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_swerveSubsystem = p_swerveSubsystem;
        m_leftJoystick = p_leftJoystick;
        m_rightJoystick = p_rightJoystick;
        m_controller = p_controller;
        addRequirements(m_swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
        @Override
    public void execute() {
        cycle++;
        if (cycle % 10 == 0)
        {
            // System.out.println("left stick y " + m_controller.getLeftY());
            // System.out.println("left stick x " + m_controller.getLeftX());

            // System.out.println("controller inputs: \nleft stick y " + m_controller.getLeftY() + 
            // "\nleft stick x " + m_controller.getLeftX() + "\nright stick x " + m_controller.getRightX()
            // + "\n \nStick inputs " + "\nleft stick y " + m_leftJoystick.getY() + "\nleft stick x " + 
            // m_leftJoystick.getX() + "\nright stick x " + m_rightJoystick.getX());

            //System.out.println(m_swerveSubsystem.getYaw());

            //forward is negative for y, backwards is positive
            //to the right is positive for x, to the left is negative
        }
        m_swerveSubsystem.swerve((m_leftJoystick.getY() * -1)+(m_controller.getLeftY() * -1), (m_leftJoystick.getX() * -1)+(m_controller.getLeftX() * -1), (m_rightJoystick.getX() * -1)+(m_controller.getRightX() * -1));
    
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
