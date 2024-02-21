package frc.robot.util;

public class Utility {
    public static boolean withinTolerance(double current, double target, double tolerance) {
        return Math.abs(current - target) <= tolerance;
    }
}
