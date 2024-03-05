package frc.robot.types;

public enum AngleSetpoint {
    min(0),
    max(1);

    public final double percentage;

    private AngleSetpoint(double percentage) {
        this.percentage = percentage;
    }
}
