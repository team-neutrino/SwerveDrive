package frc.robot;
import frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    
    public SwerveSubsystem() {
        Translation2d m_frontRightModule = new Translation2d(Swervestuff.frontRightX, Swervestuff.frontRightY);
        Translation2d m_frontLeftModule = new Translation2d(Swervestuff.frontLeftX, Swervestuff.frontLeftY);
        Translation2d m_backRightModule = new Translation2d(Swervestuff.backRightX, Swervestuff.backRightY);
        Translation2d m_backLeftModule = new Translation2d(Swervestuff.backLeftX, Swervestuff.backLeftY);

        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightModule, m_frontLeftModule, m_backRightModule, m_backLeftModule);
    }


    public void Swerve(double leftStickX, double leftStickY, SwerveDriveKinematics m_kinematics) {

        ChassisSpeeds speeds = new ChassisSpeeds(leftStickY, leftStickX, Constants.Swervestuff.angularVelocity);      

// Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

// Front left module state
    SwerveModuleState m_frontLeftModule = moduleStates[0];

// Front right module state
    SwerveModuleState _frontRightModule = moduleStates[1];

// Back left module state
    SwerveModuleState m_backLeftModule = moduleStates[2];

// Back right module state
    SwerveModuleState m_backRightModule = moduleStates[3];                                                                                                                              
    }



}
