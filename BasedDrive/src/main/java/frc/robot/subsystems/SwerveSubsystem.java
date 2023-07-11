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
    
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    
    private PIDController m_PIDSpeed;
    private PIDController m_PIDAngle;

    private SimpleMotorFeedforward m_feedForward;

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
        //continuous input, wraps around min and max (this PID controller should only be recieving normalized values)
        m_PIDAngle.enableContinuousInput(-1.0, 1.0);

        //I think we're only using feedforward for the wheel speed, not module angle
        m_feedForward = new SimpleMotorFeedforward(Constants.Swerve.Ks, Constants.Swerve.Kv);

        //SIMULATION
        SmartDashboard.putData("Field", field);
        field.getRobotObject().close();
    }

    public void Swerve(double vx, double vy, double omega) {

        //vx: input joystick Y value (left joystick)
        //vy: input joystick X value (left joystick)
        //omega: input joystick angular value (right joystick)

        //do scaling multiplication here later
        vx = Limiter.deadzone(vx, 0.1);
        vy = Limiter.deadzone(vy, 0.1);
        omega = Limiter.deadzone(omega, 0.1);

        //quick realization: somewhere in here our x and y direction needs to be multiplied by -1 (or not)
        //depending on which alliance we/the opponents are (DriverStation.getAlliance())
        //reference the fromfieldRelativeSpeeds method docs as necessary for more exact defintion of parameters and output

        //something to consider, module angle rate of change should be limited to prevent skidding when translated quickly
        //RobotCasserole uses interpolation with a look up table, might be useful

        //just reading through the navx docs, they suggest "plan for catastrophic sensor failure" by using isConnected()
        //and only using data when this is true. I can't recall ever using this in the past, but why not start now ig
        //(if we want to)

        ChassisSpeeds moduleSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(getYaw()));

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(moduleSpeeds);
        
        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        //optimization: module angle is potentially offset by 180 degrees and the wheel speed is flipped to 
        //reduce correction amount
        frontLeftState = SwerveModuleState.optimize(frontLeftState, Rotation2d.fromDegrees(m_frontLeft.getAdjustedDegrees()));
        frontRightState = SwerveModuleState.optimize(frontRightState, Rotation2d.fromDegrees(m_frontRight.getAdjustedDegrees()));
        backLeftState = SwerveModuleState.optimize(backLeftState, Rotation2d.fromDegrees(m_backLeft.getAdjustedDegrees()));
        backRightState = SwerveModuleState.optimize(backRightState, Rotation2d.fromDegrees(m_backRight.getAdjustedDegrees()));

        //should the PID calculation use getAdjustedDegrees()? This would make it 1:1 with what's being retrieved from the module states...
        //units are already the same but how many degrees is part of a real rotation is technically different for the motor and module at large
        //... 

        //ok im making the change ^^ if it doesn't work switch the methods back but I think this is correct

        //Feedforward. This will output a voltage but we are using normalized control (rn) thus the extra division
        //current gains are 0 so this step does nothing until those are changed
        double frontLeftFF = m_feedForward.calculate(frontLeftState.speedMetersPerSecond) / 12;
        double frontRightFF = m_feedForward.calculate(frontRightState.speedMetersPerSecond) / 12;
        double backLeftFF = m_feedForward.calculate(backLeftState.speedMetersPerSecond) / 12;
        double backRightFF = m_feedForward.calculate(backRightState.speedMetersPerSecond) / 12;

        //PID on wheel speeds
        double frontLeftWheelOutput = frontLeftFF + m_PIDSpeed.calculate(
            m_frontLeft.getSpeedMPS(), frontLeftState.speedMetersPerSecond);
    
        double frontRightWheelOutput = frontRightFF + m_PIDSpeed.calculate(
            m_frontRight.getSpeedMPS(), frontRightState.speedMetersPerSecond);
        
        double backLeftWheelOutput = backLeftFF + m_PIDSpeed.calculate(
            m_backLeft.getSpeedMPS(), backLeftState.speedMetersPerSecond);
        
        double backRightWheelOutput = backRightFF + m_PIDSpeed.calculate(
            m_backRight.getSpeedMPS(), backRightState.speedMetersPerSecond);

        //PID on module angle position
        double frontLeftAngleOutput = m_PIDAngle.calculate(
            m_frontLeft.getAdjustedDegrees() / 360, frontLeftState.angle.getDegrees() / 360);

        double frontRightAngleOutput = m_PIDAngle.calculate(
            m_frontRight.getAdjustedDegrees() / 360, frontRightState.angle.getDegrees() / 360);

        double backLeftAngleOutput = m_PIDAngle.calculate(
            m_backLeft.getAdjustedDegrees() / 360, backLeftState.angle.getDegrees() / 360);

        double backRightAngleOutput = m_PIDAngle.calculate(
            m_backRight.getAdjustedDegrees() / 360, backRightState.angle.getDegrees() / 360);

        //set wheel speeds (everything normalized from start to finish) (no multiplication of input rn)
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
        // newPose = swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), 
        //     new SwerveModulePosition[] {
        //         new SwerveModulePosition(m_frontLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontLeft.getDegrees())),
        //         new SwerveModulePosition(m_frontRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontRight.getDegrees())),
        //         new SwerveModulePosition(m_backLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backLeft.getDegrees())),
        //         new SwerveModulePosition(m_backRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backRight.getDegrees()))
        //     });

        // field.getObject("Sim Robot").setPose(newPose);

        //comment this out when not simulating
        //Swerve(m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
    }


}
