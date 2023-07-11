package frc.robot.Util;

import frc.robot.Util.physicsStolenFrom1736.Force2d;
import frc.robot.Util.physicsStolenFrom1736.Vector2d;
import frc.robot.Util.physicsStolenFrom1736.ForceAtPose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
//import frc.robot.PoseTelemetry;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* 
 * This is a shameless copy of Team 1736 Robot Casserole's Robot Model file. This is the only way that we know how to even attempt
 * simulation of a swerve drive. As of writing this, this is far from even running in this repo, but hopefully that will change.
 * 
 * We do not take credit for anything you see below, or phyics files for the calculations in the corresponding folder
 * 
 * - Noah, Aneesh
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class DrivetrainModel {

    ArrayList<SwerveModuleModel> modules = new ArrayList<SwerveModuleModel>();

    SimGyroSensorModel gyro;

    Field2d field;
    Pose2d dtPoseForTelemetry;
    Pose2d endRobotRefFrame = new Pose2d();

    Vector2d accel_prev = new Vector2d();
    Vector2d vel_prev   = new Vector2d();
    double   rotAccel_prev = 0;
    double   rotVel_prev   = 0;

    public DrivetrainModel(){

        modules.add(new SwerveModuleModel(Constants.FL_WHEEL_MOTOR_CANID,Constants.FL_AZMTH_MOTOR_CANID,Constants.FL_AZMTH_ENC_IDX, Constants.FL_ENCODER_MOUNT_OFFSET_RAD, false));
        modules.add(new SwerveModuleModel(Constants.FR_WHEEL_MOTOR_CANID,Constants.FR_AZMTH_MOTOR_CANID,Constants.FR_AZMTH_ENC_IDX, Constants.FR_ENCODER_MOUNT_OFFSET_RAD, true));
        modules.add(new SwerveModuleModel(Constants.BL_WHEEL_MOTOR_CANID,Constants.BL_AZMTH_MOTOR_CANID,Constants.BL_AZMTH_ENC_IDX, Constants.BL_ENCODER_MOUNT_OFFSET_RAD, false));
        modules.add(new SwerveModuleModel(Constants.BR_WHEEL_MOTOR_CANID,Constants.BR_AZMTH_MOTOR_CANID,Constants.BR_AZMTH_ENC_IDX, Constants.BR_ENCODER_MOUNT_OFFSET_RAD, true));

        gyro = new SimGyroSensorModel();

        field = new Field2d();

        dtPoseForTelemetry = new Pose2d();
    }

    public void modelReset(Pose2d pose){
        List<Transform2d> robotToModuleTF = Arrays.asList(
        new Transform2d(robotToModuleTL.get(FL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(FR), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BR), new Rotation2d(0.0)));


        endRobotRefFrame = pose;
        accel_prev = new Vector2d();
        vel_prev   = new Vector2d();
        rotAccel_prev = 0;
        rotVel_prev   = 0;
        for(int idx = 0; idx < 4; idx++){
            modules.get(idx).reset(pose.transformBy(robotToModuleTF.get(idx)));
        }
        gyro.resetToPose(pose);
    }

    public void update(boolean isDisabled, double batteryVoltage){

        Pose2d fieldReferenceFrame = new Pose2d();// global origin
        Pose2d startRobotRefFrame = endRobotRefFrame; //origin on and aligned to robot's present position in the field
        Transform2d fieldToRobotTrans = new Transform2d(fieldReferenceFrame, startRobotRefFrame);

        List<Transform2d> robotToModuleTF = Arrays.asList(
        new Transform2d(robotToModuleTL.get(FL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(FR), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BL), new Rotation2d(0.0)),
        new Transform2d(robotToModuleTL.get(BR), new Rotation2d(0.0)));

        ////////////////////////////////////////////////////////////////
        // Component-Force Calculations to populate the free-body diagram

        // Calculate each module's new position, and step it through simulation.
        for(int idx = 0; idx < 4; idx++){
            Pose2d modPose = fieldReferenceFrame.transformBy(fieldToRobotTrans).transformBy(robotToModuleTF.get(idx));
            modules.get(idx).setModulePose(modPose);
            modules.get(idx).update(isDisabled, batteryVoltage);
        }

        // Force on frame from wheel motive forces (along-tread)
        ArrayList<ForceAtPose2d> wheelMotiveForces = new ArrayList<ForceAtPose2d>(4);
        for(int idx = 0; idx < 4; idx++){
            wheelMotiveForces.add(modules.get(idx).getWheelMotiveForce());
        }

        // First half of the somewhat-dubious friction model
        Force2d preFricNetForce = new Force2d();
        wheelMotiveForces.forEach((ForceAtPose2d mf) ->{
            preFricNetForce.accum(mf.getForceInRefFrame(startRobotRefFrame)); //Add up all the forces that friction gets a chance to fight against
        });

        Force2d sidekickForce = new Force2d(0, 0);
        if(RobotController.getUserButton()){
            //Kick the robot to the side
            sidekickForce = new Force2d(0, 700);
        }

        preFricNetForce.accum(sidekickForce);

        ForceAtPose2d preFricNetForceRobotCenter = new ForceAtPose2d(preFricNetForce, startRobotRefFrame);

        // Calculate the forces from cross-tread friction at each module
        ArrayList<ForceAtPose2d> netXtreadFricForces = new ArrayList<ForceAtPose2d>(Constants.NUM_MODULES);
        for(int idx = 0; idx < 4; idx++){
            SwerveModuleModel mod = modules.get(idx);
            double perWheelForceFrac = 1.0/4; //Assume force evenly applied to all modules.
            Force2d preFricForceAtModule = preFricNetForceRobotCenter.getForceInRefFrame(mod.getModulePose()).times(perWheelForceFrac);
            netXtreadFricForces.add(mod.getCrossTreadFrictionalForce(preFricForceAtModule));
        }

        ////////////////////////////////////////////////////////////////
        // Combine forces in free-body diagram

        // Using all the above force components, do Sum of Forces
        Force2d forceOnRobotCenter = preFricNetForce;

        netXtreadFricForces.forEach((ForceAtPose2d f) -> {
            forceOnRobotCenter.accum(f.getForceInRefFrame(startRobotRefFrame));
        });
        
        ForceAtPose2d netForce = new ForceAtPose2d(forceOnRobotCenter, startRobotRefFrame);

        Force2d robotForceInFieldRefFrame = netForce.getForceInRefFrame(fieldReferenceFrame);

        robotForceInFieldRefFrame.accum(getWallCollisionForce(startRobotRefFrame));


        //Sum of Torques
        double netTorque = 0;

        for(int idx = 0; idx < 4; idx++){
            netTorque += wheelMotiveForces.get(idx).getTorque(startRobotRefFrame);
            netTorque += netXtreadFricForces.get(idx).getTorque(startRobotRefFrame);
        }


        ////////////////////////////////////////////////////////////////
        // Apply Newton's 2nd law to get motion from forces

        //a = F/m in field frame
        Vector2d accel = robotForceInFieldRefFrame.times(1/63.5).vec;

        Vector2d velocity = new Vector2d( vel_prev.x + (accel.x + accel_prev.x)/2 * 0.001, //Trapezoidal integration
                                          vel_prev.y + (accel.y + accel_prev.y)/2 * 0.001);

        Translation2d posChange = new Translation2d( (velocity.x + vel_prev.x)/2 * 0.001, //Trapezoidal integration
                                                     (velocity.y + vel_prev.y)/2 * 0.001);
        
        vel_prev = velocity;
        accel_prev = accel;
        
        //alpha = T/I in field frame
        double rotAccel = netTorque / (1.0/12.0);
        double rotVel = rotVel_prev + (rotAccel + rotAccel_prev)/2 * 0.001;
        double rotPosChange = (rotVel + rotVel_prev)/2 * 0.001;

        rotVel_prev = rotVel;
        rotAccel_prev = rotAccel;

        posChange = posChange.rotateBy(startRobotRefFrame.getRotation().unaryMinus()); //Twist needs to be relative to robot reference frame

        Twist2d motionThisLoop = new Twist2d(posChange.getX(), posChange.getY(), rotPosChange);
        
        endRobotRefFrame = startRobotRefFrame.exp(motionThisLoop);

        gyro.update(endRobotRefFrame, startRobotRefFrame);

        dtPoseForTelemetry = endRobotRefFrame;
    }

    public double getCurrentDraw(){
        double retVal = 0;
        for(int idx = 0; idx < 4; idx++){
            retVal += modules.get(idx).getCurrentDraw_A();
        }
        return retVal;
    }

    // Very rough approximation of bumpers wacking into a wall.
    // Assumes wall is a very peculiar form of squishy.
    public Force2d getWallCollisionForce(Pose2d pos_in){
        final double WALL_PUSHY_FORCE_N = 5000; 

        double FIELD_WIDTH_M = 8.2296;
        double FIELD_LENGTH_M = 16.4592;

        Translation2d MAX_ROBOT_TRANSLATION = new Translation2d(FIELD_LENGTH_M, FIELD_WIDTH_M);
        Translation2d MIN_ROBOT_TRANSLATION = new Translation2d(0.0,0.0);

        Force2d netForce_in = new Force2d();

        if(pos_in.getX() > MAX_ROBOT_TRANSLATION.getX()){
            //Too far in the positive X direction
            netForce_in = new Force2d(-WALL_PUSHY_FORCE_N, 0);
        }else if(pos_in.getX() < MIN_ROBOT_TRANSLATION.getX()){
            //Too far in the negative X direction
            netForce_in = new Force2d(WALL_PUSHY_FORCE_N, 0);
        }

        if(pos_in.getY() > MAX_ROBOT_TRANSLATION.getY()){
            //Too far in the positive Y direction
            netForce_in = new Force2d(0, -WALL_PUSHY_FORCE_N);
        }else if(pos_in.getY() < MIN_ROBOT_TRANSLATION.getY()){
            //Too far in the negative Y direction
            netForce_in = new Force2d(0, WALL_PUSHY_FORCE_N);
        }

        return netForce_in;
    }

}