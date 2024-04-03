package frc.robot.types;

public enum AngleSetpoint {
    min(0),
    half(0.5),
    podium(0.8),
    max(1);

    public final double percentage;

    private AngleSetpoint(double percentage) {
        this.percentage = percentage;
    }
}
