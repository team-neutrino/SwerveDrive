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
    //XboxController m_driverController = new XboxController(0);

    //DELETE LATER
    double cycle = 0;

    public SwerveSubsystem() {

        //needs correct dio slots
        m_frontLeft = new SwerveModule(Swerve.FLA, Swerve.FLS, 0);
        m_frontRight = new SwerveModule(Swerve.FRA, Swerve.FRS, 0);
        m_backLeft = new SwerveModule(Swerve.BLA, Swerve.BLS, 0);
        m_backRight = new SwerveModule(Swerve.BRA, Swerve.BRS, 0);

        m_PIDSpeed = new PIDController(Constants.Swerve.SPEED_P, 0, 0);
        m_PIDAngle = new PIDController(Constants.Swerve.ANGLE_P, 0, 0);
        //continuous input, wraps around min and max (this PID controller should only be recieving normalized values)
        m_PIDAngle.enableContinuousInput(0.0, 360.0);

        //I think we're only using feedforward for the wheel speed, not module angle
        m_feedForward = new SimpleMotorFeedforward(Constants.Swerve.Ks, Constants.Swerve.Kv);

        zeroYaw();

        //SIMULATION
        SmartDashboard.putData("Field", field);
        field.getRobotObject().close();
    }

    public void Swerve(double vx, double vy, double omega) {

        //vx: input joystick Y value (left joystick)
        //vy: input joystick X value (left joystick)
        //omega: input joystick angular value (right joystick)

        //deadzones stick inputs and scales + constrains chassis velocities
        vx = Limiter.joystickScale(Limiter.deadzone(vx, 0.1), -Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED, Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED);
        vy = Limiter.joystickScale(Limiter.deadzone(vy, 0.1), -Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED, Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED);
        omega = Limiter.joystickScale(Limiter.deadzone(omega, 0.1), -Constants.Swerve.MAX_CHASSIS_ROTATIONAL_SPEED, Constants.Swerve.MAX_CHASSIS_ROTATIONAL_SPEED);

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

        //I don't know if the line below will need to be used. After testing once it becomes clear how the chassis speeds transformation
        //effects module speeds we can remove it or put in place a reasonable constant. Knowing the maximum wheel speed is necessary
        //for accurate normalization however and thus hopefully useful PID (we'll see how far we can get...)
        //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);
        
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

        
        double frontLeftFF = m_feedForward.calculate(frontLeftState.speedMetersPerSecond);
        double frontRightFF = m_feedForward.calculate(frontRightState.speedMetersPerSecond);
        double backLeftFF = m_feedForward.calculate(backLeftState.speedMetersPerSecond);
        double backRightFF = m_feedForward.calculate(backRightState.speedMetersPerSecond);

        //PID on wheel speeds
        double frontLeftWheelOutput = frontLeftFF + m_PIDSpeed.calculate(
            m_frontLeft.getVelocityMPS(), frontLeftState.speedMetersPerSecond) * Constants.Swerve.Kv;
    
        double frontRightWheelOutput = frontRightFF + m_PIDSpeed.calculate(
            m_frontRight.getVelocityMPS(), frontRightState.speedMetersPerSecond) * Constants.Swerve.Kv;
        
        double backLeftWheelOutput = backLeftFF + m_PIDSpeed.calculate(
            m_backLeft.getVelocityMPS(), backLeftState.speedMetersPerSecond) * Constants.Swerve.Kv;
        
        double backRightWheelOutput = backRightFF + m_PIDSpeed.calculate(
            m_backRight.getVelocityMPS(), backRightState.speedMetersPerSecond) * Constants.Swerve.Kv;

        //PID on module angle position
        double frontLeftAngleOutput = Limiter.normalize(m_PIDAngle.calculate(
            m_frontLeft.getAbsolutePosition() * 360, frontLeftState.angle.getDegrees()), 0, 360);

        double frontRightAngleOutput = Limiter.normalize(m_PIDAngle.calculate(
            m_frontRight.getAbsolutePosition() * 360, frontRightState.angle.getDegrees()), 0, 360);

        double backLeftAngleOutput = m_PIDAngle.calculate(
            m_backLeft.getAbsolutePosition() * 360, backLeftState.angle.getDegrees());

        double backRightAngleOutput = m_PIDAngle.calculate(
            m_backRight.getAbsolutePosition() * 360, backRightState.angle.getDegrees());

        //set wheel speeds
        m_frontLeft.setWheelSpeedVolts(frontLeftWheelOutput);
        m_frontRight.setWheelSpeedVolts(frontRightWheelOutput);
        m_backLeft.setWheelSpeedVolts(backLeftWheelOutput);
        m_backRight.setWheelSpeedVolts(backRightWheelOutput);

        //set module positions speeds (in volts)
        m_frontLeft.setAngleSpeedVolts(frontLeftAngleOutput * 12.0);
        m_frontRight.setAngleSpeedVolts(frontRightAngleOutput * 12.0);
        m_backLeft.setAngleSpeedVolts(backLeftAngleOutput * 12.0);
        m_backRight.setAngleSpeedVolts(backRightAngleOutput * 12.0);
    }

    public double getYaw() {
        return m_navX.getYaw(); // * -1? I think the 2023 code has this for some reason
    }

    public double getPitch() {
        return m_navX.getPitch();
    }

    public double getRoll() {
        return m_navX.getRoll();
    }

    public void zeroYaw()
    {
        m_navX.zeroYaw();
        System.out.println("NavX yaw has been zeroed---------------");
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

        cycle++;
        if (cycle % 40 == 0)
        {
            System.out.println("Front right module velocity: " + m_frontRight.getVelocityMPS());
            System.out.println("Front left module velocity: " + m_frontLeft.getVelocityMPS());
            System.out.println("Back right module velocity: " + m_backRight.getVelocityMPS());
            System.out.println("Back left module velocity: " + m_backLeft.getVelocityMPS());

            // SwerveModuleState frontRight = new SwerveModuleState(m_frontRight.getVelocityMPS(), Rotation2d.fromDegrees(m_frontRight.getAdjustedDegrees()));
            // SwerveModuleState frontLeft = new SwerveModuleState(m_frontLeft.getVelocityMPS(), Rotation2d.fromDegrees(m_frontLeft.getAdjustedDegrees()));
            // SwerveModuleState backRight = new SwerveModuleState(m_backRight.getVelocityMPS(), Rotation2d.fromDegrees(m_backRight.getAdjustedDegrees()));
            // SwerveModuleState backLeft = new SwerveModuleState(m_backLeft.getVelocityMPS(), Rotation2d.fromDegrees(m_backLeft.getAdjustedDegrees()));

            // ChassisSpeeds forwardKinematics = m_kinematics.toChassisSpeeds(frontRight, frontLeft, backRight, backLeft);

            // System.out.println("Current chassis x direction velocity: " + forwardKinematics.vxMetersPerSecond);

            //Absolute encoder stuff
            System.out.println("Front right encoder position " + m_frontRight.getAbsolutePosition());
            System.out.println("Front left encoder position" + m_frontLeft.getAbsolutePosition());
            System.out.println("Back right encoder position " + m_backRight.getAbsolutePosition());
            System.out.println("Back left encoder position " + m_backLeft.getAbsolutePosition());
        }
    }


}
