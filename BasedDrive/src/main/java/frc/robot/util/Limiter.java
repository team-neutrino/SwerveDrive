package frc.robot.util;

public class Limiter {
    
    public static double bound(double in, double upper, double lower)
    {
        return Math.max(Math.min(in, upper), lower);
    }

    public static double bound(double in, double bound)
    {
        return bound(in, bound, -bound);
    }

    public static double deadzone(double in, double zone)
    {
        if (Math.abs(in) <= zone)
        {
            return 0;
        }
        else 
        {
            return in;
        }
    }
}
