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


       public static double frontRightX = 0.5;
       public static double frontLeftX = 0.5;
       public static double backRightX = -0.5;
       public static double backLeftX = -0.5;
       public static double frontRightY = -1;
       public static double frontLeftY = -1;
       public static double backRightY = 1;
       public static double backLeftY = 1;
        
       public static double angularVelocity = Math.PI/2;

       //PID constants
       public static double P = 0.0;
       public static double I;
       public static double D;
    }

    public static class OperatorConstants
    {
        public static int LEFT_JOYSTICK = 0;
        public static int RIGHT_JOYSTICK = 1;
    }
}
