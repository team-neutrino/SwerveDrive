// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryConfigConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutonomousUtil;
import frc.robot.util.PoseTriplet;

public class FollowTrajectoryCommand extends Command {
  /** Creates a new FollowTrajectoryCommand. */

  SwerveSubsystem m_swerve;
  HolonomicDriveController m_controller;
  Trajectory m_t;
  Timer timer = new Timer();
  Trajectory.State referenceState;
  ChassisSpeeds referenceSpeeds;
  SwerveDriveOdometry m_odometry;
  Trajectory straightTraj;
  Trajectory inftyTraj;
  Trajectory infty2Traj;


  public FollowTrajectoryCommand(SwerveSubsystem p_swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = p_swerve;
    // m_controller = p_controller;
    // m_t = p_t;
    // m_odometry = p_odometry;
    ArrayList<PoseTriplet> straightArray = new ArrayList<PoseTriplet>(Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(0.5, 0, 0), 
    new PoseTriplet(1, 0, 0)));

    ArrayList<PoseTriplet> infty = new ArrayList<PoseTriplet>(Arrays.asList(new PoseTriplet(0, 0, 0), new PoseTriplet(0.75, 0.5, 0), 
    new PoseTriplet(1.5, 0, 0), new PoseTriplet(2.25, -0.5, 0), new PoseTriplet(3, 0, 90), 
    new PoseTriplet(2.25, 0.5, 180), new PoseTriplet(1.5, 0, 180), new PoseTriplet(0.75, -0.5, 180), 
    new PoseTriplet(0, 0, 180)));
    //ArrayList<PoseTriplet> infty2 = new ArrayList<PoseTriplet>(Arrays.asList()));

    straightTraj = AutonomousUtil.generateTrajectoryFromPoses(straightArray, TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);
    inftyTraj = AutonomousUtil.generateTrajectoryFromPoses(infty, TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG);
    //infty2Traj = AutonomousUtil.generateTrajectoryFromPoses(infty2, TrajectoryConfigConstants.K_LESS_SPEED_FORWARD_CONFIG_R, true);
    // straightTraj.add(new PoseTriplet(0, 0, 0));
    // straightTraj.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("command running");
    //referenceState = m_t.sample(timer.get());
    referenceSpeeds = m_swerve.trackTrajectory(timer.get(), inftyTraj);
    m_swerve.autonSwerve(referenceSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.swerve(0, 0, 0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() >= inftyTraj.getTotalTimeSeconds())
        {
          return true;
        }
        return false;
  }
}