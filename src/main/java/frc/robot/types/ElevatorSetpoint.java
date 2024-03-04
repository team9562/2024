package frc.robot.types;

public enum ElevatorSetpoint {
    min(0),
    max(1);

    public final double percentage;

    private ElevatorSetpoint(double percentage) {
        this.percentage = percentage;
    }
}
