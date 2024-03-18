package frc.robot.util;

public class Utility {
    public static boolean withinTolerance(double current, double target, double tolerance) {
        return Math.abs(current - target) <= tolerance;
    }

    public static boolean betweenRange(double value, double min, double max) {
        return value >= min && value <= max;
    }
}
