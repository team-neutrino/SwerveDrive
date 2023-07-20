package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class SwerveModule {

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder speedEncoder;
    private AnalogEncoder absAngleEncoder;

    public SwerveModule(int angleID, int speedID, int absEncoderSlot) {

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

        absAngleEncoder = new AnalogEncoder(absEncoderSlot); //needs correct analog slot (bottom of rio)
        //it might not be a constant for every motor but for now it's set here
        //absAngleEncoder.setPositionOffset(0);

        //rpm to rps
        angleEncoder.setVelocityConversionFactor(60); //needed?
        speedEncoder.setVelocityConversionFactor(60);

        //assuming we can get abs encoders to work
        //this should make it so we can use relative encoders after the correct position has been initialized
        //oh and also make sure that the conversion between absolute encoder position and module angle is completely understood
        //because when the relative positions are set the abs:relative position is not going to be 1:1 and thus going through some
        //scaling will have to happen in order to get the relative encoders to work
        //angleEncoder.setPosition(absAngleEncoder.getAbsolutePosition() - absAngleEncoder.getPositionOffset());

        //The swerve module gear ratio is 6.55:1; dividing by this should return the module rotations in degrees
        //angleEncoder.setPositionConversionFactor(360 / 6.55); needed???
    }

    public double getRotations()
    {
        return angleEncoder.getPosition(); 
    }

    public double getAbsolutePosition()
    {
      return absAngleEncoder.getAbsolutePosition();
    }

    public double getAbsolutePositionDegrees()
    {
      return absAngleEncoder.getAbsolutePosition() * 360;
    }

    public double getDegrees()
    //in degrees
    {
        return (angleEncoder.getPosition() * 360) % 360;
    }

    public double getAdjustedDegrees()
    {
      //accounting for gear ratio? Now they should both be in degrees but are 1:1 for optimization (among other things)
      //range is (-360, 360)
      return (angleEncoder.getPosition() * 360 / 6.55) % 360; 
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

    public void setWheelSpeedVolts(double volts)
    {
      speedMotor.setVoltage(volts);
    }

    public void setAngleSpeedVolts(double volts)
    {
      angleMotor.setVoltage(volts);
    }

    public void setAngleSpeedNormalized(double in)
    {
      angleMotor.set(in);
    }
     
    // public void move(double speed, double angle) {
        
    // }

    public void resetAbsEncoder()
    {
      absAngleEncoder.reset();
    }

    public double getPositionOffset()
    {
      return absAngleEncoder.getPositionOffset();
    }
    
}
