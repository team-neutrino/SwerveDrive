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

        /*
         * This file follows the convention that the speed encoder and motor relate to the physical hardware that is 
         * responsible for the wheel on the ground spinning and the angle motor and encoder relate to the motor that controls
         * the "azimuth" in Robot Casserole terms or otherwise what we call the "module angle," or the angle that the wheel is at
         * looking at it top down. While speed AND position data is technically accessible in both cases, in reality, our control
         * process only demands one or the other depending on what our output is. As a result, our getters reflect this and only certain 
         * data is accessible from the specifc encoder in question. This may change later but the way it is formatted as of now
         * is certainly intentional, and shouldn't be overlooked if one is intending to further this code. 
         */
           
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedID, MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();
        
        angleMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setIdleMode(IdleMode.kBrake);

        angleEncoder = angleMotor.getEncoder();
        speedEncoder = speedMotor.getEncoder();

        //rpm to rps
        angleEncoder.setVelocityConversionFactor(60); //needed?
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

    public double getAdjustedDegrees()
    {
      //accounting for gear ratio? Now they should both be in degrees but are 1:1 for optimization (among other things)
      return (angleEncoder.getPosition() * 360 / 7.36) % 360; 
    }

    public double getVelocityRPS()
    {
      //default returns RPM, conversion is 60 so it should be returning
      //rotations per second
      return speedEncoder.getVelocity();
    }

    public double getVelocityMPS()
    {
      //returns the wheel speeds in m/s instead of r/s
      return speedEncoder.getVelocity() * Constants.DimensionConstants.WHEEL_DIAMETER_M;
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
