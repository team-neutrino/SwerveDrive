package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveSubsystem extends SubsystemBase {
    
    private CANSparkMax m_frontLeftAngle =
    new CANSparkMax(Swervestuff.FLA, MotorType.kBrushless);
    private CANSparkMax m_frontRightAngle =
    new CANSparkMax(Swervestuff.FRA, MotorType.kBrushless);
    private CANSparkMax m_backLeftAngle =
    new CANSparkMax(Swervestuff.BLA, MotorType.kBrushless);
    private CANSparkMax m_backRightAngle =
    new CANSparkMax(Swervestuff.BRA, MotorType.kBrushless);

    private CANSparkMax m_frontLeftSpeed =
    new CANSparkMax(Swervestuff.FLS, MotorType.kBrushless);
    private CANSparkMax m_frontRightSpeed =
    new CANSparkMax(Swervestuff.FRS, MotorType.kBrushless);
    private CANSparkMax m_backLeftSpeed =
    new CANSparkMax(Swervestuff.BLS, MotorType.kBrushless);
    private CANSparkMax m_backRightSpeed =
    new CANSparkMax(Swervestuff.BRS, MotorType.kBrushless);

    private RelativeEncoder m_encoderFLA;
    private RelativeEncoder m_encoderFRA;
    private RelativeEncoder m_encoderBLA;
    private RelativeEncoder m_encoderBRA;
    private RelativeEncoder m_encoderFLS;
    private RelativeEncoder m_encoderFRS;
    private RelativeEncoder m_encoderBLS;
    private RelativeEncoder m_encoderBRS;

    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    public SwerveSubsystem(Joystick p_leftJoystick, Joystick p_rightJoystick) {

        m_encoderFLA = initializeMotor(m_frontLeftAngle);
        m_encoderFRA = initializeMotor(m_frontRightAngle);
        m_encoderBLA = initializeMotor(m_backLeftAngle);
        m_encoderBRA = initializeMotor(m_backRightAngle);
        m_encoderFLS = initializeMotor(m_frontLeftSpeed);
        m_encoderFRS = initializeMotor(m_frontRightSpeed);
        m_encoderBLS = initializeMotor(m_backLeftSpeed);
        m_encoderBRS = initializeMotor(m_backRightSpeed);
        
    }


    private RelativeEncoder initializeMotor(CANSparkMax p_motor) {
        RelativeEncoder p_encoder;

        p_motor.restoreFactoryDefaults();
        p_motor.setIdleMode(IdleMode.kBrake);

        p_encoder = p_motor.getEncoder();
        return p_encoder;
  }



    Translation2d m_frontRightModule = new Translation2d(Swervestuff.frontRightX, Swervestuff.frontRightY);
    Translation2d m_frontLeftModule = new Translation2d(Swervestuff.frontLeftX, Swervestuff.frontLeftY);
    Translation2d m_backRightModule = new Translation2d(Swervestuff.backRightX, Swervestuff.backRightY);
    Translation2d m_backLeftModule = new Translation2d(Swervestuff.backLeftX, Swervestuff.backLeftY);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightModule, m_frontLeftModule, m_backRightModule, m_backLeftModule);


    public void Swerve(double strafe, double forward, double angle) {

        ChassisSpeeds moduleSpeeds = new ChassisSpeeds(forward, strafe, angle);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(moduleSpeeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
    }



}
