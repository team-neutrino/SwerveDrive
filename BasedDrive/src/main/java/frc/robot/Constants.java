package frc.robot;

public class Constants {
    
    public static class Swerve {
       public static int FLA = 1;
       public static int FRA = 2;
       public static int BLA = 3; 
       public static int BRA = 4;
       public static int FLS = 5;
       public static int FRS = 6;
       public static int BLS = 7;
       public static int BRS = 8;


       public static double frontRightX = 0.2155261469;
       public static double frontLeftX = 0.2155261469;
       public static double backRightX = -0.2155261469;
       public static double backLeftX = -0.2155261469;
       public static double frontRightY = -0.2155261469;
       public static double frontLeftY = -0.2155261469;
       public static double backRightY = 0.2155261469;
       public static double backLeftY = 0.2155261469;
        
       public static double angularVelocity = Math.PI/2;

       //Max velocities in m/s or rad/s (for rotation)
       public static double MAX_CHASSIS_LINEAR_SPEED = 1;
       public static double MAX_CHASSIS_ROTATIONAL_SPEED = Math.PI / 2;
       public static double MAX_MODULE_ROTATION_SPEED;

       //PID constants
       //Note that robot casserole's units are way different than ours so the commented values are likely VERY arbitrary
       public static double SPEED_P = 0.0; //robot casserole uses 0.08
       public static double ANGLE_P = 0.0; //robot casserole uses 0.015. 
       public static double I;
       public static double D;

       //Feedforward constants
       public static double Ks = 0.1; //0.1 - 0.15 seems to be a good starting place, possibly lower. Robot casserole uses 0.15
       public static double Kv = 0.39; //robot casserole uses 0.02, so maybe 0.01?
       //public static double Ka = 0.0; needed?

       //Gear ratio
       //This ratio can vary widely (5.5:1 - 7.8:1) according to the docs. I know we went with belt, but the exact ratio
       //is still unknown to me. I'm hoping this is correct but we'll have to check with design
       public static double MODULE_ANGLE_GEAR_RATIO = 1/6.55;
    }

    public static class OperatorConstants
    {
        public static int LEFT_JOYSTICK = 0;
        public static int RIGHT_JOYSTICK = 1;
        public static int XBOX_CONTROLLER = 2;
    }

    public static class DimensionConstants
    {
        //if I'm not mistaken, 4.064 inches
        public static double WHEEL_DIAMETER_M = 0.1032256;
    }
}
