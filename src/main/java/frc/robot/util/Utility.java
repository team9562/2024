package frc.robot.util;

public class Utility {
    public static boolean withinTolerance(double current, double target, double tolerance) {
        return Math.abs(current - target) <= tolerance;
    }

    public static String stripFileExtension(final String s) {
        return s != null && s.lastIndexOf(".") > 0 ? s.substring(0, s.lastIndexOf(".")) : s;
    }

    public static boolean betweenRange(double value, double min, double max) {
        return value >= min && value <= max;
    }
}
