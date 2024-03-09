package frc.robot.types;

public enum IntakeState {
    idle("idle"),
    intake("intake"),
    exhaust("exhaust");

    private final String text;

    private IntakeState(String text) {
        this.text = text;
    }

    public String toString() {
        return text;
    }
}
