
package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.*;
import frc.robot.util.Limiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;


public class SwerveSubsystem extends SubsystemBase {

    //swerve drive kinematics constants
    Translation2d frontLeft = new Translation2d(Constants.Swerve.frontLeftX, Constants.Swerve.frontLeftX);
    Translation2d frontRight = new Translation2d(Constants.Swerve.frontRightX, Constants.Swerve.frontRightX);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeft, frontRight);

    AHRS m_navX = new AHRS(SPI.Port.kMXP);

    SwerveModule m_frontLeft = new SwerveModule(Constants.Swerve.FLA, Constants.Swerve.FLS);

    public SwerveSubsystem() {
        
       
    }

    public void Swerve(double vx, double vy, double omega) {

        vx = Limiter.deadzone(vx, 0.1);
        vy = Limiter.deadzone(vy, 0.1);
        omega = Limiter.deadzone(omega, 0.1);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, omega);

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d.fromDegrees(getYaw()));

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];

        frontLeftState = SwerveModuleState.optimize(frontLeftState, Rotation2d.fromDegrees(m_frontLeft.getAbsAngle()));

        
    }

    public double getYaw()
    {
        return m_navX.getYaw() * -1;
    }
}
    