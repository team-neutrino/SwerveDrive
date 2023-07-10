package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
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
    
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    
    private PIDController m_PIDSpeed;
    private PIDController m_PIDAngle;

    private AHRS m_navX = new AHRS(SPI.Port.kMXP);

    Translation2d m_frontRightModule = new Translation2d(Swerve.frontRightX, Swerve.frontRightY);
    Translation2d m_frontLeftModule = new Translation2d(Swerve.frontLeftX, Swerve.frontLeftY);
    Translation2d m_backRightModule = new Translation2d(Swerve.backRightX, Swerve.backRightY);
    Translation2d m_backLeftModule = new Translation2d(Swerve.backLeftX, Swerve.backLeftY);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightModule, m_frontLeftModule, m_backRightModule, m_backLeftModule);

    //SIMULATION
    Pose2d newPose = new Pose2d(3, 3, Rotation2d.fromDegrees(0));
    SwerveModulePosition[] swervePositions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};
    SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0), swervePositions, newPose);
    Field2d field = new Field2d();
    XboxController m_driverController = new XboxController(0);

    public SwerveSubsystem() {

        m_frontLeft = new SwerveModule(Swerve.FLA, Swerve.FLS);
        m_frontRight = new SwerveModule(Swerve.FRA, Swerve.FRS);
        m_backLeft = new SwerveModule(Swerve.BLA, Swerve.BLS);
        m_backRight = new SwerveModule(Swerve.BRA, Swerve.BRS);

        m_PIDSpeed = new PIDController(Constants.Swerve.P, 0, 0);
        m_PIDAngle = new PIDController(Constants.Swerve.P, 0, 0);

        //SIMULATION
        SmartDashboard.putData("Field", field);
        field.getRobotObject().close();
    }

    public void Swerve(double vx, double vy, double omega) {

        //vx: input joystick Y value
        //vy: input joystick X value
        //omega: input joystick angular value

        //quick realization: somewhere in here our x and y direction needs to be multiplied by -1 (or not)
        //depending on which alliance we/the opponents are (DriverStation.getAlliance())
        //reference the fromfieldRelativeSpeeds method docs as necessary for more exact defintion of parameters and output

        //something to consider, module angle rate of change should be limited to prevent skidding when translated quickly
        //RobotCasserole uses interpolation with a look up table, might be useful

        ChassisSpeeds moduleSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(getYaw()));

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(moduleSpeeds);
        
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];


        //PID on wheel speeds
        double frontLeftWheelOutput = m_PIDSpeed.calculate(
            m_frontLeft.getSpeed(), frontLeftState.speedMetersPerSecond);
    
        double frontRightWheelOutput = m_PIDSpeed.calculate(
            m_frontRight.getSpeed(), frontRightState.speedMetersPerSecond);
        
        double backLeftWheelOutput = m_PIDSpeed.calculate(
            m_backLeft.getSpeed(), backLeftState.speedMetersPerSecond);
        
        double backRightWheelOutput = m_PIDSpeed.calculate(
            m_backRight.getSpeed(), backRightState.speedMetersPerSecond);

        //PID on module angle position
        double frontLeftAngleOutput = m_PIDAngle.calculate(
            m_frontLeft.getDegrees() / 360, frontLeftState.angle.getDegrees() / 360);

        double frontRightAngleOutput = m_PIDAngle.calculate(
            m_frontRight.getDegrees() / 360, frontRightState.angle.getDegrees() / 360);

        double backLeftAngleOutput = m_PIDAngle.calculate(
            m_backLeft.getDegrees() / 360, backLeftState.angle.getDegrees() / 360);

        double backRightAngleOutput = m_PIDAngle.calculate(
            m_backRight.getDegrees() / 360, backRightState.angle.getDegrees() / 360);

        //set wheel speeds
        m_frontLeft.setSpeedVelocity(frontLeftWheelOutput);
        m_frontRight.setSpeedVelocity(frontRightWheelOutput);
        m_backLeft.setSpeedVelocity(backLeftWheelOutput);
        m_backRight.setSpeedVelocity(backRightWheelOutput);

        //set module positions speeds (in volts)
        m_frontLeft.setSpeedVolts(frontLeftAngleOutput * 12.0);
        m_frontRight.setSpeedVolts(frontRightAngleOutput * 12.0);
        m_backLeft.setSpeedVolts(backLeftAngleOutput * 12.0);
        m_backRight.setSpeedVolts(backRightAngleOutput * 12.0);
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
        newPose = swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), 
            new SwerveModulePosition[] {
                new SwerveModulePosition(m_frontLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontLeft.getDegrees())),
                new SwerveModulePosition(m_frontRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontRight.getDegrees())),
                new SwerveModulePosition(m_backLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backLeft.getDegrees())),
                new SwerveModulePosition(m_backRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backRight.getDegrees()))
            });

        field.getObject("Sim Robot").setPose(newPose);

        //comment this out when not simulating
        Swerve(m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
    }


}
