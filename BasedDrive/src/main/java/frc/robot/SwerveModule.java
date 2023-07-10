package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;


public class SwerveModule {
    public SwerveModule(int angleID, int speedID) {
           
        CANSparkMax angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);
        CANSparkMax speedMotor = new CANSparkMax(speedID, MotorType.kBrushless);

        angleMotor.restoreFactoryDefaults();
        angleMotor.setIdleMode(IdleMode.kBrake);

        RelativeEncoder angleEncoder = angleMotor.getEncoder();

        speedMotor.restoreFactoryDefaults();
        speedMotor.setIdleMode(IdleMode.kBrake);

        RelativeEncoder speedEncoder = angleMotor.getEncoder();

        public void move(double speed, double angle) {
            
        }

  }
    
}
