// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * Some of the numbers were taken from last years code. I have no clue what's "fast" or not as of writing this but 
 * I assume that all of this will be tuned and adjusted as needed.
 */

/** Add your docs here. */
public class TrajectoryConfigConstants {

    public static final TrajectoryConfig K_LESS_SPEED_FORWARD_CONFIG = new TrajectoryConfig(1, 1)
    .setReversed(false)
    ;
    //in the 2023 code other things were set here like the kinematics and other voltage constraints. There is no voltage constraint class for swerve drives however
    //I think one could make the argument that this is redudant because low level control (voltage) is handled through the modules anyways. I think this is fine for now.
}
