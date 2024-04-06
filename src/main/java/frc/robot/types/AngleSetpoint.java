package frc.robot.types;

import frc.robot.Constants.AngleConstants;

public enum AngleSetpoint {
    min(0),
    half(0.5),
    podium(43.48 / AngleConstants.ANGLE_MAX_REL),
    max(1);

    public final double percentage;

    private AngleSetpoint(double percentage) {
        this.percentage = percentage;
    }
}
