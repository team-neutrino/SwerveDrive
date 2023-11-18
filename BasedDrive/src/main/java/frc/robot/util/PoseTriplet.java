// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/**
 * Written by David Dong, not myself (Noah Williamson)
 */

public class PoseTriplet {
  private double m_coord1;
  private double m_coord2;
  private double m_angle;

  public PoseTriplet(double p_coord1, double p_coord2, double p_angle) {
    m_coord1 = p_coord1;
    m_coord2 = p_coord2;
    m_angle = p_angle;
  }

  public double getCoord1() {
    return m_coord1;
  }

  public double getCoord2() {
    return m_coord2;
  }

  public double getAngle() {
    return m_angle;
  }

  public void setCoord1(double p_coord1) {
    m_coord1 = p_coord1;
  }

  public void setCoord2(double p_coord2) {
    m_coord2 = p_coord2;
  }

  public void setAngle(double p_angle) {
    m_angle = p_angle;
  }
}