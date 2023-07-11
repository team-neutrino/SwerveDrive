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

       //PID constants
       public static double P = 0.0;
       public static double I;
       public static double D;

       //Feedforward constants
       public static double Ks = 0.0;
       public static double Kv = 0.0;
       //public static double Ka = 0.0; needed?
    }

    public static class OperatorConstants
    {
        public static int LEFT_JOYSTICK = 0;
        public static int RIGHT_JOYSTICK = 1;
        public static int XBOX_CONTROLLER;
    }

    public static class DimensionConstants
    {
        //if I'm not mistaken, 4.064 inches
        public static double WHEEL_DIAMETER_M = 0.1032256;
    }
}
