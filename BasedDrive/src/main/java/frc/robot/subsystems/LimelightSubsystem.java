// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable limelight;
  private int cycle = 0;
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

  /* is a print that access and prints the full array that is accessed when getting the camTran. Inert for right now,
   * may be used in the shuffleboard subsystem so left here at the wish of Cale.
   */
  public void printCamTran() {
    if (cycle % 40 == 0) {
      double[] camTran = getCamTran();
      // System.out.println("Translation X: " + camTran[0]);
      // System.out.println("Translation Y: " + camTran[1]);
      // System.out.println("Translation Z: " + camTran[2]);
      // System.out.println("Rotation Pitch: " + camTran[3]);
      // System.out.println("Rotation Yall: " + camTran[4]);
      // System.out.println("Rotation Roll: " + camTran[5]);
    }
  }

  @Override
  public void periodic() {
    cycle++;
    printCamTran();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
