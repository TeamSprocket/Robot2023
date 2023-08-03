package frc.util;

public class Conversions {

    public static double InchesToMeters(double inches) {
        double meters = inches * 0.0254;
        return meters;
    }

    public static double TicksToMeters(double ticks, double gearRatio, double radius) {
        double pos = ticks / gearRatio;
        double circum = 2 * Math.PI * radius;
        double meters = pos * circum;
        return meters;
    }
    
    public static double TicksToDegrees(double ticks, double gearRatio) {
        return ticks / 2048.0 / gearRatio * 360.0;
    }

    public static double RotationsToDegrees(double rotations, double gearRatio) {
        return rotations / gearRatio * 360.0;
    }
    
}
