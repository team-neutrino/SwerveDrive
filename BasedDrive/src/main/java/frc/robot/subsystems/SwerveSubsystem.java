
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

        // System.out.println("vx " + vx);
        // System.out.println("vy " + vy);

        // final double CURRENT_YAW = getYaw();
        // omega += m_PIDAngle.calculate( CURRENT_YAW, CURRENT_YAW + omega);
       
        // System.out.println("omega " + omega);
        
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
                    // System.out.println("last omega " + lastOmega + "\nlast angle " + lastAngle + "\ncurrent angle " + getYaw());
                    // System.out.println("error " + (lastAngle - getYaw()));

                    // System.out.println("last wheel speed " + lastWheelSpeed + "\nlast angle " + lastAngle + "\ncurrent angle " + getYaw());
                    // System.out.println("error " + (lastAngle - getYaw()));
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

        //ChassisSpeeds moduleSpeedsTwo = new ChassisSpeeds(vx, vy, omega);

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

        //System.out.println("front right state angle " + frontRightState.angle.getDegrees());

        //should the PID calculation use getAdjustedDegrees()? This would make it 1:1 with what's being retrieved from the module states...
        //units are already the same but how many degrees is part of a real rotation is technically different for the motor and module at large
        //... 

        //ok im making the change ^^ if it doesn't work switch the methods back but I think this is correct

        
        double frontLeftFF = m_feedForward.calculate(frontLeftState.speedMetersPerSecond);
        double frontRightFF = m_feedForward.calculate(frontRightState.speedMetersPerSecond);
        double backLeftFF = m_feedForward.calculate(backLeftState.speedMetersPerSecond);
        double backRightFF = m_feedForward.calculate(backRightState.speedMetersPerSecond);

        //PID on wheel speeds
        // double frontLeftWheelOutput = frontLeftFF + m_PIDSpeed.calculate(
        //     m_frontLeft.getVelocityMPS(), frontLeftState.speedMetersPerSecond) * Constants.Swerve.Kv;
    
        // double frontRightWheelOutput = frontRightFF + m_PIDSpeed.calculate(
        //     m_frontRight.getVelocityMPS(), frontRightState.speedMetersPerSecond) * Constants.Swerve.Kv;
        
        // double backLeftWheelOutput = backLeftFF + m_PIDSpeed.calculate(
        //     m_backLeft.getVelocityMPS(), backLeftState.speedMetersPerSecond) * Constants.Swerve.Kv;
        
        // double backRightWheelOutput = backRightFF + m_PIDSpeed.calculate(
        //     m_backRight.getVelocityMPS(), backRightState.speedMetersPerSecond) * Constants.Swerve.Kv;

        //PID on module angle position
        // double frontLeftAngleOutput = m_PIDAngle.calculate(
        //     m_frontLeft.getAbsolutePosition() * 360, frontLeftState.angle.getDegrees());

        // double frontRightAngleOutput = m_PIDAngle.calculate(
        //     m_frontRight.getAbsolutePosition() * 360, frontRightState.angle.getDegrees());

        // double backLeftAngleOutput = m_PIDAngle.calculate(
        //     m_backLeft.getAbsolutePosition() * 360, backLeftState.angle.getDegrees());

        // double backRightAngleOutput = m_PIDAngle.calculate(
        //     m_backRight.getAbsolutePosition() * 360, backRightState.angle.getDegrees());

        //set wheel speeds
        // m_frontLeft.setWheelSpeedVolts(frontLeftWheelOutput);
        // m_frontRight.setWheelSpeedVolts(frontRightWheelOutput);
        // m_backLeft.setWheelSpeedVolts(backLeftWheelOutput);
        // m_backRight.setWheelSpeedVolts(backRightWheelOutput);

        //set module position speeds (in volts)
        // m_frontLeft.setAngleSpeedVolts(frontLeftAngleOutput * 12.0);
        // m_frontRight.setAngleSpeedVolts(frontRightAngleOutput * 12.0);
        // m_backLeft.setAngleSpeedVolts(backLeftAngleOutput * 12.0);
        // m_backRight.setAngleSpeedVolts(backRightAngleOutput * 12.0);

        if (cycle % 8 == 0)
        {
            //System.out.println("back right reference " + backRightState.angle.getDegrees());  
            // System.out.println("vx " + vx);
            // System.out.println("vy " + vy);
            //System.out.println("reference " + 90);
        }

        //System.out.println("back right reference " + backRightState.angle.getDegrees()); 
        // System.out.println("vx " + vx);
        // System.out.println("vy " + vy); 

        //m_backRight.runAnglePID(90);

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
        if(!angleAlignOn) //if it's off turn it on 
        {
            angleAlignOn = true;
            System.out.println("angle alignment is on ----------------");
        }
        else
        {
            angleAlignOn = false;
            System.out.println("angle alignment is off -----------------");
        }
    }

    // public void resetAllModuleAbsEncoders()
    // {
    //     m_frontRight.resetAbsEncoder();
    //     m_frontLeft.resetAbsEncoder();
    //     m_backRight.resetAbsEncoder();
    //     m_backLeft.resetAbsEncoder();
    //     System.out.println("abs encoders reset-----------------");
    // }

    @Override
    public void periodic()
    {
        //AUTON
        frontRightPosition = m_frontRight.getPosition();
        frontLeftPosition = m_frontLeft.getPosition();
        backRightPosition = m_backRight.getPosition();
        backLeftPosition = m_backLeft.getPosition();

        autonPose = swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), swervePositions);

        //System.out.println("angle error between odometry and navx " + (swerveOdometry.getPoseMeters().getRotation().getDegrees() - getYaw()));
        //System.out.println("swerveOdomety angle " + swerveOdometry.getPoseMeters().getRotation().getDegrees());


        // newPose = swerveOdometry.update(Rotation2d.fromDegrees(getYaw()), 
        //     new SwerveModulePosition[] {
        //         new SwerveModulePosition(m_frontLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontLeft.getDegrees())),
        //         new SwerveModulePosition(m_frontRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_frontRight.getDegrees())),
        //         new SwerveModulePosition(m_backLeft.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backLeft.getDegrees())),
        //         new SwerveModulePosition(m_backRight.getRotations() * DimensionConstants.WHEEL_DIAMETER_M, Rotation2d.fromDegrees(m_backRight.getDegrees()))
        //     });

        field.getObject("real robot").setPose(autonPose);

        //comment this out when not simulating
        //Swerve(m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());

        // System.out.println("Back right encoder position " + m_backRight.getAbsolutePosition());
        // System.out.println("Back right encoder position adjusted " + m_backRight.getAbsolutePositionAdjusted());
        //System.out.println("Back right encoder voltage " + m_backRight.getAbsEncoderVoltage());

        

        cycle++;
        if (cycle % 8 == 0)
        {
            //System.out.println("angle error between odometry and navx " + (swerveOdometry.getPoseMeters().getRotation().getDegrees() - getAdjustedYaw()));

            //System.out.println("odometry angle " + swerveOdometry.getPoseMeters().getRotation().getDegrees());
            //System.out.println("navX angle " + getYaw());


            speedTest += Math.abs(m_frontRight.getVelocityMPS()) + Math.abs(m_frontLeft.getVelocityMPS()) + Math.abs(m_backRight.getVelocityMPS()) + Math.abs(m_backLeft.getVelocityMPS());
            speedTest /= 4;
            //System.out.println("average wheel speed " + speedTest);
            //System.out.println("counts per rotation " + m_backRight.countsPerRotation());

            //System.out.println("incremental encoder measurement period " + m_backLeft.getMeasurementPeriod());

            // System.out.println("Front right module velocity: " + m_frontRight.getVelocityMPS());
            // System.out.println("Front left module velocity: " + m_frontLeft.getVelocityMPS());
            // System.out.println("Back right module velocity: " + m_backRight.getVelocityMPS());
            //System.out.println("Back left module velocity: " + m_backLeft.getVelocityMPS());
            //System.out.println("counts " + m_backLeft.countsPerRotation());

            //System.out.println("navX angle " + getYaw());

            //none of the below forward kinematics will work until the proper conversion for position is done. It's not hard,
            //but does require some if statements and saving off values and I'm too lazy to type it all out now since it might not ever be used
            // SwerveModuleState frontRight = new SwerveModuleState(m_frontRight.getVelocityMPS(), Rotation2d.fromDegrees(m_frontRight.getAbsolutePosition()));
            // SwerveModuleState frontLeft = new SwerveModuleState(m_frontLeft.getVelocityMPS(), Rotation2d.fromDegrees(m_frontLeft.getAbsolutePosition()));
            // SwerveModuleState backRight = new SwerveModuleState(m_backRight.getVelocityMPS(), Rotation2d.fromDegrees(m_backRight.getAbsolutePosition()));
            // SwerveModuleState backLeft = new SwerveModuleState(m_backLeft.getVelocityMPS(), Rotation2d.fromDegrees(m_backLeft.getAbsolutePosition()));

            // ChassisSpeeds forwardKinematics = m_kinematics.toChassisSpeeds(frontRight, frontLeft, backRight, backLeft);

            // System.out.println("Current chassis x direction velocity: " + forwardKinematics.vxMetersPerSecond);

            //Absolute encoder stuff
            //System.out.println("Front right encoder position " + m_frontRight.getAbsolutePosition()); // 338.7 degrees = 0
            //System.out.println("front right encoder voltage " + m_frontRight.getAbsEncoderVoltage());
            //System.out.println("Front left encoder position " + m_frontLeft.getAbsolutePosition()); // 40.8
            //System.out.println("front left encoder voltage " + m_frontLeft.getAbsEncoderVoltage());
            //System.out.println("Back right encoder position " + m_backRight.getAbsolutePosition());
            //System.out.println("back right encoder voltage " + m_backRight.getAbsEncoderVoltage());
            //System.out.println("Back left encoder position " + m_backLeft.getAbsolutePosition()); // 82.7
            //System.out.println("back left encoder voltage " + m_backLeft.getAbsEncoderVoltage());

            //System.out.println("front right wheel distance " + m_frontRight.getWheelDistance());
            //System.out.println("front left wheel distance " + m_frontLeft.getWheelDistance());
        }
    }


}
