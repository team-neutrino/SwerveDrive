package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    
    private Joystick m_leftJoystick;
    private Joystick m_rightJoystick;

    private PIDController m_PIDSpeed;
    private PIDController m_PIDAngle;

    private AHRS m_navX = new AHRS(SPI.Port.kMXP);

    Translation2d m_frontRightModule = new Translation2d(Swerve.frontRightX, Swerve.frontRightY);
    Translation2d m_frontLeftModule = new Translation2d(Swerve.frontLeftX, Swerve.frontLeftY);
    Translation2d m_backRightModule = new Translation2d(Swerve.backRightX, Swerve.backRightY);
    Translation2d m_backLeftModule = new Translation2d(Swerve.backLeftX, Swerve.backLeftY);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightModule, m_frontLeftModule, m_backRightModule, m_backLeftModule);

    public SwerveSubsystem(Joystick p_leftJoystick, Joystick p_rightJoystick) {

        m_frontLeft = new SwerveModule(Swerve.FLA, Swerve.FLS);
        m_frontRight = new SwerveModule(Swerve.FRA, Swerve.FRS);
        m_backLeft = new SwerveModule(Swerve.BLA, Swerve.BLS);
        m_backRight = new SwerveModule(Swerve.BRA, Swerve.BRS);

        m_PIDSpeed = new PIDController(Constants.Swerve.P, 0, 0);
        m_PIDAngle = new PIDController(Constants.Swerve.P, 0, 0);
    }

    public void Swerve(double vx, double vy, double omega) {

        //vx: input joystick Y value
        //vy: input joystick X value
        //omega: input joystick angular value

        ChassisSpeeds moduleSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(getYaw()));

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(moduleSpeeds);
        
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];
    }

    public double getYaw() {
        return m_navX.getYaw();
    }

    public double getPitch() {
        return m_navX.getPitch();
    }

    public double getRoll() {
        return m_navX.getRoll();
    }

    @Override
    public void periodic()
    {
        
    }


}
