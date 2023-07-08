package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class SwerveSubsystem extends SubsystemBase {
    
    private CANSparkMax m_motorLeft1 =
    new CANSparkMax(Constants.MOTOR_LEFT1, MotorType.kBrushless);
    private CANSparkMax m_motorLeft2 =
    new CANSparkMax(Constants.MOTOR_LEFT2, MotorType.kBrushless);
    private CANSparkMax m_motorRight1 =
    new CANSparkMax(Constants.MOTOR_RIGHT1, MotorType.kBrushless);
    private CANSparkMax m_motorRight2 =
    new CANSparkMax(Constants.MOTOR_RIGHT2, MotorType.kBrushless);

    private RelativeEncoder m_encoderLeft1;
    private RelativeEncoder m_encoderLeft2;
    private RelativeEncoder m_encoderRight1;
    private RelativeEncoder m_encoderRight2;
    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    public SwerveSubsystem() {
        
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
