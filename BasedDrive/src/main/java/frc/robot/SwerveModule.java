package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.Limiter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
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
    private int angleID;

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

        this.angleID = angleID;
           
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedID, MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        speedMotor.restoreFactoryDefaults();
        
        angleMotor.setIdleMode(IdleMode.kBrake);
        speedMotor.setIdleMode(IdleMode.kBrake);

        if (speedID != 5)
        {
          speedMotor.setInverted(true);
        }

        angleEncoder = angleMotor.getEncoder();
        speedEncoder = speedMotor.getEncoder();

        //somewhere in here the position conversion factor has to be set with the abs encoder so that the pid is getting the same
        //units for its reference and actual input

        absAngleEncoder = angleMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        absAngleEncoder.setInverted(true);
        //(0, 360) now instead of (0, 3.3)
        absAngleEncoder.setPositionConversionFactor(360 / 3.3);
        speedEncoder.setVelocityConversionFactor(60);
        //divide by the gear ratio to convert to wheel rotations, multiply by circumference to get meters
        //this conversion seems to be correct
        speedEncoder.setPositionConversionFactor(Constants.DimensionConstants.WHEEL_CIRCUMFERENCE_M / 6.55);

        anglePIDController = angleMotor.getPIDController();
        anglePIDController.setFeedbackDevice(absAngleEncoder);
        anglePIDController.setP(Constants.Swerve.ANGLE_P, 0);
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMaxInput(360);
        anglePIDController.setPositionPIDWrappingMinInput(0);

        speedPIDController = speedMotor.getPIDController();
        speedPIDController.setFeedbackDevice(speedEncoder);
        speedPIDController.setP(Constants.Swerve.SPEED_P, 0);


        //needed for finding position offset?
        //angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


        //it might not be a constant for every motor but for now it's set here
        //absAngleEncoder.setZeroOffset(0);

        //rpm to rps
        
        speedEncoder.setVelocityConversionFactor(50 / (4096 * 6.55)); //* 4096 * 6.55)); //needed?

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

    public double getWheelDistance()
    {
      return speedEncoder.getPosition();
    }

    public SwerveModulePosition getPosition()
    {
      return new SwerveModulePosition(getWheelDistance(), Rotation2d.fromDegrees(getAbsolutePosition()));
    }

    public double getAbsolutePosition()
    {
      return adjustAngleOut(absAngleEncoder.getPosition());
    }

    //used to convert back to wpilib system for angle optimization (don't delete)
    public double getAdjustedAbsolutePosition()
    {
      double pos = getAbsolutePosition();
      if (pos <= 180)
      {
        return pos * -1;
      }
      else 
      {
        return 360 - pos;
      }
    }

    /**
     * Helper method that takes in the desired angle (most likely a reference position) for use with the angle motor and encoder and applies the correct
     * position adjustments as necessary to account for known offset. 
     * @param angle The input angle to be used by the angle hardware
     * @return Adjusted angle that has been modified for known offset
     */
    public double adjustAngleIn(double angle)
    {
      if (angleID == 2)
      {
        if (angle < 277.3)
            {
                angle += 82.7;
            }
            else 
            {
                angle = angle + 82.7 - 360;
            }
      }

      //The back right module (angleID = 4) has no need of offset. Nice.

      else if (angleID == 6)
      {
        if (angle < 21.3)
            {
                angle = angle + 360 - 21.3;
            }
            else 
            {
                angle -= 21.3;
            }
      }

      else if (angleID == 8)
      {
        if (angle < 319.2)
            {
                angle += 40.8;
            }
            else 
            {
                angle = angle + 40.8 - 360;
            }
      }

      return angle;
    }

    /**
     * Helper method that takes in the raw position from the absolute analog encoder that is present in the module and adjusts the retrieved angle as
     * necessary to account for known offsets.
     * @param angle The initial position (0, 360) (increasing clockwise) from the absolute encoder that has not been adjusted for offsets
     * @return Accurate angle that reflects the real module angle after accounting for offsets
     */
    public double adjustAngleOut(double angle)
    {
      if (angleID == 2)
      {
        if (angle < 82.7)
        {
          angle = angle - 82.7 + 360;
        }
        else 
        {
          angle -= 82.7;
        }
      }

      //The back right module (angleID = 4) has no need of offset. Nice.

      else if (angleID == 6)
      {
        if (angle > 338.7)
        {
          angle = angle + 21.3 - 360;
        }
        else 
        {
          angle += 21.3;
        }
      }

      else if (angleID == 8)
      {
        if (angle < 40.8)
        {
          angle = angle - 40.8 + 360;
        }
        else 
        {
          angle -= 40.8;
        }
      }

      return angle;
    }

    public double getVelocityRaw()
    {
      return speedEncoder.getVelocity();
    }

    public int countsPerRotation()
    {
      return speedEncoder.getCountsPerRevolution();
    }

    public int getMeasurementPeriod()
    {
      return speedEncoder.getMeasurementPeriod();
    }

    public double getVelocityMPS()
    {
      //returns the wheel speeds in m/s instead of rotations/s
      return speedEncoder.getVelocity() * Constants.DimensionConstants.WHEEL_CIRCUMFERENCE_M;
    }

    // public void setAngle(double speed)
    // {
    //   speedMotor.set(speed);
    // }

    // public void setSpeedVelocity(double speed)
    // {
    //   speedMotor.set(speed);
    // }

    // public void setWheelSpeedVolts(double volts)
    // {
    //   speedMotor.setVoltage(volts);
    // }

    // public void setAngleSpeedVolts(double volts)
    // {
    //   angleMotor.setVoltage(volts);
    // }

    // public void setAngleSpeedNormalized(double in)
    // {
    //   angleMotor.set(in);
    // }

    public void runAnglePID(double reference)
    {
      reference = adjustAngleIn(reference);
      anglePIDController.setReference(reference, ControlType.kPosition, 0);
    }

    public void runSpeedPID(double reference, double feedForward)
    {
      speedPIDController.setReference(reference, CANSparkMax.ControlType.kVelocity, 0, feedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public double getAbsEncoderVoltage()
    {
      return absAngleEncoder.getVoltage();
    }

    public double getAbsRawPosition()
    {
      return absAngleEncoder.getPosition();
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
