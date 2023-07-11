package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder speedEncoder;

    public SwerveModule(int angleID, int speedID) {
           
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedID, MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();
        
        angleMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder = angleMotor.getEncoder();
        speedEncoder = angleMotor.getEncoder();

        //rpm to rps
        angleEncoder.setVelocityConversionFactor(60);
        speedEncoder.setVelocityConversionFactor(60);

        //The swerve module gear ratio is 7.36:1; dividing by this should return the module rotations
        //angleEncoder.setPositionConversionFactor(1/7.36); needed???
    }

    public double getRotations()
    {
        return angleEncoder.getPosition(); 
    }

    public double getDegrees()
    //in degrees
    {
        return (angleEncoder.getPosition() * 360) % 360;
    }

    public double getSpeed()
    {
      //default returns RPM, conversion is 60 so it should be returning
      //rotations per second
      return speedEncoder.getVelocity();
    }

    // public void setAngle(double speed)
    // {
    //   speedMotor.set(speed);
    // }

    public void setSpeedVelocity(double speed)
    {
      speedMotor.set(speed);
    }

    public void setSpeedVolts(double speed)
    {
      angleMotor.setVoltage(speed);
    }
     
    // public void move(double speed, double angle) {
        
    // }

    
}
