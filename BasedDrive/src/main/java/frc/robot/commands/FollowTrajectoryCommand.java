// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
  /** Creates a new FollowTrajectoryCommand. */

  SwerveSubsystem m_swerve;
  HolonomicDriveController m_controller;
  Trajectory m_t;
  Timer timer = new Timer();
  Trajectory.State state;
  ChassisSpeeds reference;
  SwerveDriveOdometry m_odometry;


  public FollowTrajectoryCommand(SwerveSubsystem p_swerve, HolonomicDriveController p_controller, Trajectory p_t, SwerveDriveOdometry p_odometry) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = p_swerve;
    m_controller = p_controller;
    m_t = p_t;
    m_odometry = p_odometry;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    state = m_t.sample(timer.get());
    reference = m_controller.calculate()
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
