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

    /**
     * Normalizes the input value to a new value between [0, 1]
     * @param in input value
     * @param min minimum possible input
     * @param max maximum possible input
     * @return normalized value between [0, 1]
     */
    public static double normalize(double in, double min, double max)
    {
        return (in - min) / (max - min);
    }

    /**
     * Reverses the normalization process. Assumes that the input value is normalized to [0, 1]
     * @param in input value
     * @param min minimum possible value that is desired for output
     * @param max maximum possible value that is desired for output
     * @return A reverse normalized output that is scaled to the min and max values
     */
    public static double inverseNormalize(double in, double min, double max)
    {
        return in * (max - min) + min;
    }

    /**
     * Inverse normalization but assumes the input value is between [-1, 1]
     * @param in input value
     * @param min minimum possible value that is desired for output
     * @param max maximum possible value that is desired for output
     * @return A reverse normalized value that is scaled to the min and max values
     */
    public static double joystickInverseNormalize(double in, double min, double max)
    {
        return (in + 1) * (max - min) / 2 + min;
    }
}
