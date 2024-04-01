package frc.robot.types;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutonConstants.PPSpeakerPositions;

public enum SpeakerPosition {
    blueMiddle(PPSpeakerPositions.MIDDLE_BLUE),
    blueSourceSide(PPSpeakerPositions.SOURCE_SIDE_BLUE),
    blueAmpSide(PPSpeakerPositions.AMP_SIDE_BLUE),
    
    redMiddle(PPSpeakerPositions.MIDDLE_RED),
    redSourceSide(PPSpeakerPositions.SOURCE_SIDE_RED),
    redAmpSide(PPSpeakerPositions.AMP_SIDE_RED);

    public final Pose2d pose;

    private SpeakerPosition(Pose2d pose) {
        this.pose = pose;
    }
}
