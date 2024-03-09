package frc.robot.types;

public enum ElevatorSetpoint {
    min(0),
    hang(0),
    half(0.5),
    max(1);

    public final double percentage;

    private ElevatorSetpoint(double percentage) {
        this.percentage = percentage;
    }
}
