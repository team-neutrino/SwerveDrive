// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.TrajectoryConfigConstants;
// import frc.robot.subsystems.DriveTrainSubsystem;
import java.util.ArrayList;

/**
 * Written by David Dong, not myself (Noah Williamson)
 * 
 * has undergone minor edits to interface with this codebase better instead of 2023 robot code
 */

public class AutonomousUtil {

//   public static RamseteCommand generateRamseteCommand(
//       Trajectory trajectory, DriveTrainSubsystem p_drivetrainSubsystem) {
//     return new RamseteCommand(
//         trajectory,
//         p_drivetrainSubsystem::getPose2d,
//         new RamseteController(
//             TrajectoryConfigConstants.K_RAMSETE_BETA, TrajectoryConfigConstants.K_RAMSETE_ZETA),
//         new SimpleMotorFeedforward(
//             TrajectoryConfigConstants.KS_VOLTS,
//             TrajectoryConfigConstants.KV_VOLT_SECONDS_PER_METER),
//         TrajectoryConfigConstants.K_DRIVE_KINEMATICS,
//         p_drivetrainSubsystem::getDriveWheelSpeeds,
//         new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
//         new PIDController(TrajectoryConfigConstants.KP_DRIVE_VEL, 0, 0),
//         p_drivetrainSubsystem::setVoltage,
//         p_drivetrainSubsystem);
//   }

  public static Trajectory generateTrajectoryFromPoses(
      ArrayList<PoseTriplet> tripletList, TrajectoryConfig config) {
    ArrayList<Pose2d> poseArray = new ArrayList<Pose2d>();
    for (PoseTriplet triplet : tripletList) {
      double coord1 = triplet.getCoord1();
      double coord2 = triplet.getCoord2();
      double angle = triplet.getAngle();
      poseArray.add(new Pose2d(coord1, coord2, Rotation2d.fromDegrees(angle)));
    }
    return TrajectoryGenerator.generateTrajectory(poseArray, config);
  }

  public static Trajectory generateTrajectoryFromPoses(
      ArrayList<PoseTriplet> tripletList, TrajectoryConfig config, boolean inverted) {
    ArrayList<Pose2d> poseArray = new ArrayList<Pose2d>();
    for (PoseTriplet triplet : tripletList) {
      double coord1 = triplet.getCoord1();
      double coord2 = triplet.getCoord2();
      double angle = triplet.getAngle();
      if (inverted) {
        coord2 = -coord2;
        angle = -angle;
      }
      poseArray.add(new Pose2d(coord1, coord2, Rotation2d.fromDegrees(angle)));
    }
    return TrajectoryGenerator.generateTrajectory(poseArray, config);
  }

//   public static RamseteCommand generateRamseteFromPoses(
//       ArrayList<PoseTriplet> tripletList,
//       DriveTrainSubsystem p_drivetrainSubsystem,
//       TrajectoryConfig trajectoryConfig,
//       boolean inverted) {
//     Trajectory generatedTrajectory =
//         generateTrajectoryFromPoses(tripletList, trajectoryConfig, inverted);
//     RamseteCommand generatedRamseteCommand =
//         generateRamseteCommand(generatedTrajectory, p_drivetrainSubsystem);
//     return generatedRamseteCommand;
//   }

//   public static RamseteCommand generateRamseteFromPoses(
//       ArrayList<PoseTriplet> tripletList,
//       DriveTrainSubsystem p_drivetrainSubsystem,
//       TrajectoryConfig trajectoryConfig) {
//     Trajectory generatedTrajectory = generateTrajectoryFromPoses(tripletList, trajectoryConfig);
//     RamseteCommand generatedRamseteCommand =
//         generateRamseteCommand(generatedTrajectory, p_drivetrainSubsystem);
//     return generatedRamseteCommand;
//   }

//   public static RamseteCommand generateRamseteFromPoseFile(
//       String poseFilename, DriveTrainSubsystem p_drivetrainSubsystem) {
//     ArrayList<PoseTriplet> tripletList = PoseProcessor.poseTripletsFromFile(poseFilename);
//     Trajectory generatedTrajectory =
//         generateTrajectoryFromPoses(
//             tripletList, TrajectoryConfigConstants.K_MAX_SPEED_FORWARD_CONFIG);
//     RamseteCommand generatedRamseteCommand =
//         generateRamseteCommand(generatedTrajectory, p_drivetrainSubsystem);
//     return generatedRamseteCommand;
//   }
}