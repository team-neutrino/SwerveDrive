package frc.robot;

public class Constants {
    
    public static class Swerve {
       public static int FLA;
       public static int FRA;
       public static int BLA; 
       public static int BRA;
       public static int FLS;
       public static int FRS;
       public static int BLS;
       public static int BRS;


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
}
