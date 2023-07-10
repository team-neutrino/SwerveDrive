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
    }

        public double getSpeed()
        {
          //default returns RPM, conversion is 60 so it should be returning
          //rotations per second
          return speedEncoder.getVelocity();
        }

        public double getAngle()
        {
          //default returns rotations, 
          return angleEncoder.getPosition();
        }

        public void move(double speed, double angle) {
            
        }

    
}
