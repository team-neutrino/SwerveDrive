// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable limelight;
  private double LIMELIGHT_TO_METER_CONVERSION = 0.76189;

  public LimelightSubsystem() {
    // global instance of the network table and gets the limelight table
    limelight = NetworkTableInstance.getDefault().getTable("limelight");

    // turns off LED
    limelight.getEntry("ledMode").setNumber(1);
  }
  

  // return whether or not the limelight sees a target
  public boolean getTv() {
    NetworkTableEntry tv = limelight.getEntry("tv");
    double validTarget = tv.getDouble(0.0);
    if (validTarget == 1) {
      return true;
    }
    return false;
  }

  // gets ID of the april tag that is detected
  public double getID() {
    return limelight.getEntry("tid").getDouble(0.0);
  }

  // gets the x offest between the center of vision and the detected object
  public double getTx() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  // gets the y offest between the center of vision and the detected object
  public double getTy() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public double testTransformDistance(double tx) {
    double distanceFromLimelight = Math.tan(tx);
    return distanceFromLimelight;
  }

  public double[] getCamTran() {
    return limelight.getEntry("camtran").getDoubleArray(new double[] {});
  }

  public double getDistance() {
    double[] camTran = getCamTran();
    return camTran[2] * LIMELIGHT_TO_METER_CONVERSION;
  }


  @Override
  public void periodic() {
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
