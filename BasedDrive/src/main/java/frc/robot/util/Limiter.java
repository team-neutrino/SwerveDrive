package frc.robot.util;

public class Limiter {

    public static double deadzone(double in, double zone)
    {
        if (Math.abs(in) < zone)
        {
            return 0.0;
        }
        return in;
    }

    public static double scale(double in)
    {
        return 2 * ((in - 0) / (2));
    }
}