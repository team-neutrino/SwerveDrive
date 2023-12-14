
package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.*;
import frc.robot.util.Limiter;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;


public class SwerveSubsystem extends SubsystemBase {
    
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    
    // private PIDController m_PIDSpeed;
    private PIDController m_PIDAngle;

    private SimpleMotorFeedforward m_feedForward;

    private AHRS m_navX = new AHRS(SPI.Port.kMXP);

    Translation2d m_frontRightModule = new Translation2d(Swerve.frontRightX, Swerve.frontRightY);
    Translation2d m_frontLeftModule = new Translation2d(Swerve.frontLeftX, Swerve.frontLeftY);
    Translation2d m_backRightModule = new Translation2d(Swerve.backRightX, Swerve.backRightY);
    Translation2d m_backLeftModule = new Translation2d(Swerve.backLeftX, Swerve.backLeftY);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightModule, m_frontLeftModule, m_backRightModule, m_backLeftModule);

    SwerveModuleState[] moduleStates;

    SwerveModuleState frontLeftState;
    SwerveModuleState frontRightState;
    SwerveModuleState backLeftState;
    SwerveModuleState backRightState;

    double[] pastModuleAngles = new double[4];

    SwerveModuleState frontLeftStatePast;
    SwerveModuleState frontRightStatePast;
    SwerveModuleState backLeftStatePast;
    SwerveModuleState backRightStatePast;

    boolean angleAlignOn = false;

    //AUTON
    SwerveModulePosition frontRightPosition;
    SwerveModulePosition frontLeftPosition;
    SwerveModulePosition backRightPosition;
    SwerveModulePosition backLeftPosition;
    Pose2d autonPose = new Pose2d();
    //SwerveModulePosition[] swervePositions = {frontRightPosition, frontLeftPosition, backRightPosition, backLeftPosition};
    SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
    SwerveDriveOdometry swerveOdometry;
    HolonomicDriveController controller;
    
   double lastAngle = 0;
   double angleOut = 0;
   double lastOmega = 0;
   double vFactor = 0;
   boolean omegaOnPrev = false;
   double speedTest = 0;

   double lastWheelSpeed = 0;
   double[] lastWheelSpeeds = new double[4];

    //SIMULATION
    Field2d field = new Field2d();
    //XboxController m_driverController = new XboxController(0);

    //DELETE LATER
    double cycle = 0;

    public SwerveSubsystem() {

        m_frontLeft = new SwerveModule(Swerve.FLA, Swerve.FLS);
        m_frontRight = new SwerveModule(Swerve.FRA, Swerve.FRS);
        m_backLeft = new SwerveModule(Swerve.BLA, Swerve.BLS);
        m_backRight = new SwerveModule(Swerve.BRA, Swerve.BRS);

        // m_PIDSpeed = new PIDController(Constants.Swerve.SPEED_P, 0, 0);
        m_PIDAngle = new PIDController(0.1, 0, 0);
        // //continuous input, wraps around min and max (this PID controller should only be recieving normalized values)
        m_PIDAngle.enableContinuousInput(-180, 180);

        //I think we're only using feedforward for the wheel speed, not module angle
        m_feedForward = new SimpleMotorFeedforward(Constants.Swerve.Ks, Constants.Swerve.Kv);

        

        //AUTON
        frontRightPosition = new SwerveModulePosition(0, Rotation2d.fromDegrees(m_frontRight.getAdjustedAbsolutePosition()));
        frontLeftPosition = new SwerveModulePosition(0, Rotation2d.fromDegrees(m_frontLeft.getAdjustedAbsolutePosition()));
        backRightPosition = new SwerveModulePosition(0, Rotation2d.fromDegrees(m_backRight.getAdjustedAbsolutePosition()));
        backLeftPosition = new SwerveModulePosition(0, Rotation2d.fromDegrees(m_backLeft.getAdjustedAbsolutePosition()));

        swervePositions[0] = frontRightPosition;
        swervePositions[1] = frontLeftPosition;
        swervePositions[2] = backRightPosition;
        swervePositions[3] = backLeftPosition;

        autonPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getYaw()));
        // swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(getYaw()), swervePositions, autonPose);

        Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d middle = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
        Pose2d end = new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(0));

        TrajectoryConfig config = new TrajectoryConfig(1, 1);

        ArrayList<Pose2d> arr = new ArrayList<Pose2d>();

        arr.add(start);
        arr.add(middle);
        arr.add(end);

        Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(arr, config);

        controller = new HolonomicDriveController
        (new PIDController(0.1, 0, 0), new PIDController(0.1, 0, 0), 
        new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

        swerveOdometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(getYaw()), swervePositions, autonPose);

        //SIMULATION
        SmartDashboard.putData("Field", field);
        field.getRobotObject().close();

        zeroYaw();
        
        AutoBuilder.configureHolonomic(this::getPose, this::resetPose, this::getSpeeds, this::driveRobotRelative,Constants.PathPlannerConstants.pathFollowerConfig , this);
    }

    public void swerve(double vx, double vy, double omega) {

        //vx: input joystick Y value (left joystick)
        //vy: input joystick X value (left joystick)
        //omega: input joystick angular value (right joystick)

        //deadzones stick inputs and scales + constrains chassis velocities
        vx = Limiter.joystickScale(Limiter.deadzone(vx, 0.2), -Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED, Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED);
        vy = Limiter.joystickScale(Limiter.deadzone(vy, 0.2), -Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED, Constants.Swerve.MAX_CHASSIS_LINEAR_SPEED);
        omega = Limiter.joystickScale(Limiter.deadzone(omega, 0.2), -Constants.Swerve.MAX_CHASSIS_ROTATIONAL_SPEED, Constants.Swerve.MAX_CHASSIS_ROTATIONAL_SPEED);

        //quick realization: somewhere in here our x and y direction needs to be multiplied by -1 (or not)
        //depending on which alliance we/the opponents are (DriverStation.getAlliance())
        //reference the fromfieldRelativeSpeeds method docs as necessary for more exact defintion of parameters and output

        //something to consider, module angle rate of change should be limited to prevent skidding when translated quickly
        //RobotCasserole uses interpolation with a look up table, might be useful

        //just reading through the navx docs, they suggest "plan for catastrophic sensor failure" by using isConnected()
        //and only using data when this is true. I can't recall ever using this in the past, but why not start now ig
        //(if we want to)

        
        if (angleAlignOn)
        {
            if (omega == 0)
            {
                //System.out.println("omega is zero ");

                vFactor = (13.6066 * (lastWheelSpeed) - 0.9628);

                //double vFactor2 = 16 * (lastWheelSpeed) - 4;

                if (lastOmega > 0)
                {
                angleOut = m_PIDAngle.calculate(getYaw(), lastAngle + vFactor);
                }
                else
                {
                    angleOut = m_PIDAngle.calculate(getYaw(), lastAngle - vFactor);
                }

                //angleOut = m_PIDAngle.calculate(getYaw(), lastAngle + vFactor);
                omega += angleOut;
                //System.out.println("navx angle " + getYaw());
                if (cycle % 10 == 0)
                {
                }
            }
            else
            {
                lastAngle = getYaw();
                lastOmega = omega;

                lastWheelSpeeds[0] = m_frontRight.getVelocityMPS();
                lastWheelSpeeds[1] = m_frontLeft.getVelocityMPS();
                lastWheelSpeeds[2] = m_backLeft.getVelocityMPS();
                lastWheelSpeeds[3] = m_backRight.getVelocityMPS();

                for (int i = 0; i < 4; i++)
                {
                    if (lastWheelSpeeds[i] < 0) lastWheelSpeeds[i] *= -1;

                    lastWheelSpeed += lastWheelSpeeds[i];
                }

                lastWheelSpeed /= 4;

                //lastWheelSpeed = m_backLeft.getVelocityMPS();
                //System.out.println("last angle " + lastAngle);

            }
        }

        ChassisSpeeds moduleSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(getYaw()));

        moduleStates = m_kinematics.toSwerveModuleStates(moduleSpeeds);

        //optimization: module angle is potentially offset by 180 degrees and the wheel speed is flipped to 
        //reduce correction amount

        moduleStates[1] = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(m_frontLeft.getAdjustedAbsolutePosition()));
        moduleStates[0] = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(m_frontRight.getAdjustedAbsolutePosition()));
        moduleStates[3] = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(m_backLeft.getAdjustedAbsolutePosition()));
        moduleStates[2] = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(m_backRight.getAdjustedAbsolutePosition()));

        for (int i = 0; i < 4; i++)
        {
            if (moduleStates[i].angle.getDegrees() <= 0)
            {
                moduleStates[i].angle = Rotation2d.fromDegrees(moduleStates[i].angle.getDegrees() * -1);
            }
            else 
            {
                moduleStates[i].angle = Rotation2d.fromDegrees(360 - moduleStates[i].angle.getDegrees());
            }
        }

        if (vx != 0 || vy != 0 || omega != 0)
        {
            for (int i = 0; i < 4; i++)
            {
                pastModuleAngles[i] = moduleStates[i].angle.getDegrees();
            }
        }  

        if (vx == 0 && vy == 0 && omega == 0)
            {
                for (int i = 0; i < 4; i++)
                {
                    moduleStates[i].angle = Rotation2d.fromDegrees(pastModuleAngles[i]);
                    moduleStates[i].speedMetersPerSecond = 0;
                }
            }

        //I don't know if the line below will need to be used. After testing once it becomes clear how the chassis speeds transformation
        //effects module speeds we can remove it or put in place a reasonable constant. Knowing the maximum wheel speed is necessary
        //for accurate normalization however and thus hopefully useful PID (we'll see how far we can get...)
        //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);
        
        frontLeftState = moduleStates[1];
        frontRightState = moduleStates[0];
        backLeftState = moduleStates[3];
        backRightState = moduleStates[2];

      
        
        double frontLeftFF = m_feedForward.calculate(frontLeftState.speedMetersPerSecond);
        double frontRightFF = m_feedForward.calculate(frontRightState.speedMetersPerSecond);
        double backLeftFF = m_feedForward.calculate(backLeftState.speedMetersPerSecond);
        double backRightFF = m_feedForward.calculate(backRightState.speedMetersPerSecond);

       

        m_frontLeft.runAnglePID(frontLeftState.angle.getDegrees());
        m_frontRight.runAnglePID(frontRightState.angle.getDegrees());
        m_backLeft.runAnglePID(backLeftState.angle.getDegrees());
        m_backRight.runAnglePID(backRightState.angle.getDegrees());

        m_frontLeft.runSpeedPID(frontLeftState.speedMetersPerSecond, frontLeftFF);
        m_frontRight.runSpeedPID(frontRightState.speedMetersPerSecond, frontRightFF);
        m_backLeft.runSpeedPID(backLeftState.speedMetersPerSecond, backLeftFF);
        m_backRight.runSpeedPID(backRightState.speedMetersPerSecond, backRightFF);
    }

    public ChassisSpeeds trackTrajectory(double timeStamp, Trajectory t)
    {
        Trajectory.State referenceState = t.sample(timeStamp);
        return controller.calculate(autonPose, referenceState, Rotation2d.fromDegrees(0));
    }

    public double getYaw() {
        return m_navX.getYaw() * -1; // INCLUDE THE NEGATIVE ONE it is needed to make sure that the negatives go the right way
        //navX default system is opposite what all of wpilib uses, where negative is on the right from the center I think
    }

    public double getAdjustedYaw()
    {
        double yaw = m_navX.getYaw();
        if (yaw < 0)
        {
            return yaw * -1;
        }
        else
        {
            return 360 - yaw;
        }
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
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(0), swervePositions, new Pose2d());
        lastAngle = 0;
        lastOmega = 0;
    }

    public void toggleAngleAlign()
    {
        if(!angleAlignOn)
        {
            angleAlignOn = true;
        }
        else
        {
            angleAlignOn = false;
        }

        if (angleAlignOn)
        {
            System.out.println("angle alignment is on ----------------");
        }
        else
        {
            System.out.println("angle alignment is off-----------------");
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), swervePositions, pose);
    }
    public ChassisSpeeds getSpeeds() {
        return m_kinematics.toChassisSpeeds(moduleStates);
    }
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, Rotation2d.fromDegrees(getYaw())); 
    }

    @Override
    public void periodic()
    {
        //AUTON
        swervePositions[0] = m_frontRight.getPosition();
        swervePositions[1] = m_frontLeft.getPosition();
        swervePositions[2] = m_backRight.getPosition();
        swervePositions[3] = m_backLeft.getPosition();

        autonPose = swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), swervePositions);

        field.getObject("real robot").setPose(autonPose);
        

        cycle++;
        if (cycle % 8 == 0)
        {


            speedTest += Math.abs(m_frontRight.getVelocityRaw()) + Math.abs(m_frontLeft.getVelocityRaw()) + Math.abs(m_backRight.getVelocityRaw()) + Math.abs(m_backLeft.getVelocityRaw());
            speedTest /= 4;

            System.out.println("autonPose x " + autonPose.getX() + " autonPose y " + autonPose.getY());
        }
    }


}
