package frc.robot.types;

public enum InOutDirection {
    in(1),
    out(-1);

    public final double percentage;

    private InOutDirection(double percentage) {
        this.percentage = percentage;
    }
}
