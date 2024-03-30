package frc.robot.types;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutonConstants.PPSpeakerPositions;

public enum SpeakerPosition {
    middle(PPSpeakerPositions.MIDDLE),
    sourceSide(PPSpeakerPositions.SOURCE_SIDE),
    ampSide(PPSpeakerPositions.AMP_SIDE);

    public final Pose2d pose;

    private SpeakerPosition(Pose2d pose) {
        this.pose = pose;
    }
}
