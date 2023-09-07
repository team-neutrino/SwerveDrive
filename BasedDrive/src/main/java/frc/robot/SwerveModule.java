package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.Limiter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {

    int angleID;
    int speedID;

    CANSparkMax m_speedMotor;
    CANSparkMax m_angleMotor;

    SparkMaxAnalogSensor m_absEncoder;
    

    public SwerveModule(int angleID, int speedID) {
      this.angleID = angleID;
      this.speedID = speedID;

      m_speedMotor = new CANSparkMax(speedID, MotorType.kBrushless);
      m_angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        
      m_absEncoder = m_angleMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);

      
    }

    public double getAbsAngle()
    {
      return m_absEncoder.getPosition() * 360; 
    }


  }

    