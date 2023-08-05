package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder speedEncoder;
    private SparkMaxAnalogSensor absAngleEncoder;
    private SparkMaxPIDController anglePIDController;
    private SparkMaxPIDController speedPIDController;

    //PID gain interpolation points
    double[][] points = {{0, 5, 180}, {0.0001, 0.0002, 0.0005}};
    // double x0 = 15;
    // double y0 = 0.0003;
    // double x1 = 180;
    // double y1 = 0.0005;

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

        //somewhere in here the position conversion factor has to be set with the abs encoder so that the pid is getting the same
        //units for its reference and actual input

        absAngleEncoder = angleMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        //(0, 360) now instead of (0, 3.3)
        absAngleEncoder.setPositionConversionFactor(360 / 3.3);
        speedEncoder.setVelocityConversionFactor(60);

        anglePIDController = angleMotor.getPIDController();
        anglePIDController.setFeedbackDevice(absAngleEncoder);
        anglePIDController.setP(Constants.Swerve.ANGLE_P);
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMaxInput(360);
        anglePIDController.setPositionPIDWrappingMinInput(0);

        speedPIDController = speedMotor.getPIDController();
        speedPIDController.setFeedbackDevice(speedEncoder);
        speedPIDController.setP(Constants.Swerve.SPEED_P);


        //needed for finding position offset?
        //angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


        //it might not be a constant for every motor but for now it's set here
        //absAngleEncoder.setZeroOffset(0);

        //rpm to rps
        angleEncoder.setVelocityConversionFactor(60); //needed?

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
      return absAngleEncoder.getPosition();
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
      return speedEncoder.getVelocity() * Constants.DimensionConstants.WHEEL_RADIUS_M;
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

    public void runAnglePID(double reference)
    {
      //scheduleAnglePIDGains(reference, getAbsolutePosition());
      anglePIDController.setReference(reference, ControlType.kPosition);
    }

    public void runSpeedPID(double reference, double feedForward)
    {
      //what is the pidSlot???
      speedPIDController.setReference(reference, CANSparkMax.ControlType.kVelocity, 0, feedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void scheduleAnglePIDGains(double reference, double currentPos)
    {
      double p;
      double error = Math.abs(reference - currentPos);
      if (error >= 5)
      {
        // (y0 * (x1 - error) + y1 * (error - x0)) / (x1 - x0)
        //p = (points[1][0] * (points[0][1] - error) + points[1][1] * (error - points[0][0])) / (points[0][1] - points[0][0]);
        //p = (points[1][1] * (points[0][2] - error) + points[1][2] * (error - points[0][1])) / (points[0][2] - points[0][1]);
        p = 0.0005;
      }
      else 
      {
        p = 0;
      }
      anglePIDController.setP(p);
      
    }

    public double getAbsEncoderVoltage()
    {
      return absAngleEncoder.getVoltage();
    }
     
    // public void move(double speed, double angle) {
        
    // }

    // public void resetAbsEncoder()
    // {
    //   absAngleEncoder.reset();
    // }

    // public double getPositionOffset()
    // {
    //   return absAngleEncoder.getPositionOffset();
    // }
    
}
