package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = Units.lbsToKilograms(120.8);
  public static final Matter CHASSIS = new Matter(new Translation3d(Units.inchesToMeters(28), Units.inchesToMeters(32), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  
  public static final class MotorConstants { 
    public static final int NEO_550_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_550_FREE_LIMIT = 1;
    
    public static final int NEO_V1_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_V1_FREE_LIMIT = 1;
    
    public static final int NEO_V1_MAX_RPMS = 5600; // 5676, this is a safe limit
    public static final int NEO_550_MAX_RPMS = 10000; // 11K, this is a safe limit
  }
  
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0,0.125);
    public static final PIDConstants ANGLE_PID = new PIDConstants(2.75, 0, 0.055);

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4.4196, 5, 322.415, 720.93);
    public static final double ROTATION_DELAY_METERS = 0.35;

    public static final class VantagePoints {
      public static final Pose2d PP_H3_VANTAGE_POINT_BLUE = new Pose2d(1.75, 4.125, Rotation2d.fromDegrees(0));
      public static final Pose2d PP_H3_VANTAGE_POINT_RED = new Pose2d(15, 4.1, Rotation2d.fromDegrees(180));
     
      public static final Pose2d PP_SHOOTING_BLUE = new Pose2d(2.70, 5.55, Rotation2d.fromDegrees(180));
      public static final Pose2d PP_SHOOTING_RED = new Pose2d(13.85, 5.55, Rotation2d.fromDegrees(0));
     
      public static final Pose2d PP_C5_VANTAGE_POINT_BLUE = new Pose2d(7.15, 0.8, Rotation2d.fromDegrees(0));
      public static final Pose2d PP_C5_VANTAGE_POINT_RED = new Pose2d(9.55, 0.8, Rotation2d.fromDegrees(180));

      public static final Pose2d PP_C4_VANTAGE_POINT_BLUE = new Pose2d(7.15, 2.1, Rotation2d.fromDegrees(0));
      public static final Pose2d PP_C4_VANTAGE_POINT_RED = new Pose2d(9.55, 2.1, Rotation2d.fromDegrees(180));
    }

    public static final class PPSpeakerPositions {
      public static final Pose2d MIDDLE_BLUE = new Pose2d(1.45, 5.55, Rotation2d.fromDegrees(180));
      public static final Pose2d MIDDLE_BLUE_BACK = new Pose2d(1.75, 5.55, Rotation2d.fromDegrees(180));
      public static final Pose2d SOURCE_SIDE_BLUE = new Pose2d(0.75, 4.35, Rotation2d.fromDegrees(120));
      public static final Pose2d AMP_SIDE_BLUE = new Pose2d(0.75, 6.75, Rotation2d.fromDegrees(-120));
      
      public static final Pose2d MIDDLE_RED = new Pose2d(15.1, 5.55, Rotation2d.fromDegrees(0));
      public static final Pose2d MIDDLE_RED_BACK = new Pose2d(14.85, 5.55, Rotation2d.fromDegrees(0));
      public static final Pose2d SOURCE_SIDE_RED = new Pose2d(15.75, 4.35, Rotation2d.fromDegrees(60));
      public static final Pose2d AMP_SIDE_RED = new Pose2d(15.75, 6.75, Rotation2d.fromDegrees(-60));
    }
  }
  
  public static final class DrivebaseConstants {
    public static final double LOCK_TIME = 10; // seconds

    public static final class TurnAroundPIDConstants {
      //public static final double kP = 0.0505;
      public static final double kP = 0.055;
      public static final double kI = 0.00028;
      public static final double kD = 0;
      public static final double TOLERANCE = 4.5;
    }
  }
  
  public static final class ElevatorConstants {
    public static final double SPROCKET_RADIUS = 4.5;
    public static final double GEAR_RATIO = 16;

    public static final int LEFT_CAN = 13;
    public static final int RIGHT_CAN = 14;

    public static final int LIMIT_SWITCH_PORT = 9;
    
    public static final double MAX_HEIGHT = 62;
    public static final double MIN_HEIGHT = 0;
    
    public static final double kP = 0.1;
    public static final double kP_HANG = 0.01;
    public static final double kI = 0;
    public static final double kD = 0.0625;
    public static final double kFF = 0.0001;
    public static final double kFF_HANG = 0.0004;
    
    public static final int PID_SLOT = 0;
    public static final int PID_SLOT_HANG = 1;
    
    public static final int STALL_LIMIT = 50;
    
    public static final double ELEVATOR_THRESHOLD = 0.25;
    
    public static final double SAFE_MIN_PERCENT = 0.065;
    public static final double SAFE_MAX_PERCENT = 0.824;
  }
  
  public static final class ShooterConstants {
    public static final int LEFT_CAN = 15;
    public static final int RIGHT_CAN = 16;
    public static final int FEEDER_CAN = 17;

    public static final double FEED_DURATION = 0.125;

    public static final double RPM_THRESHOLD = 5000;
    
    public static final int LIMIT_SWITCH_PORT = 7;
    
    public static final double kP_LEFT = 1;
    public static final double kI_LEFT = 0.2;
    public static final double kD_LEFT = 0;
    public static final double kFF_LEFT = 0.0006;

    public static final double kP_RIGHT = 1;
    public static final double kI_RIGHT = 0.25;
    public static final double kD_RIGHT = 0;
    public static final double kFF_RIGHT = 0.0006;

    public static final double kP_FEEDER = 0.000125;
    public static final double kI_FEEDER = 0;
    public static final double kD_FEEDER = 0.125;
    public static final double kFF_FEEDER = 0.0001;
    
    public static final int STALL_LIMIT = 40;
    public static final int FEEDER_STALL_LIMIT = 25;
    
    public static final double SENSOR_THRESHOLD_INCHES = 0.5;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_CAN = 18;
    
    public static final int STALL_LIMIT = 25;
  }
  
  public static final class AngleConstants {
    public static final int ANGLE_CAN = 19;
    
    public static final double ANGLE_MAX_ABS = -0.543;
    public static final double ANGLE_MIN_ABS = 0.555;
    
    public static final double ANGLE_MIN_REL = 0;
    public static final double ANGLE_MAX_REL = 53.5;
    
    public static final double ANGLE_OFFSET_FLAT = 21; // degrees

    public static final int ENCODER_PORT = 8;
    
    public static final double kP = 0.0315;
    public static final double kI = 0;
    public static final double kD = 0.0625;
    public static final double kFF = 0.0003;

    public static final int STALL_LIMIT = 35;

    public static final double ANGLE_THRESHOLD = 2;
  }

  public static final class VisionConstants {
    public static final class NotesConstants {
      public static final String TABLE_KEY = "limelight-notes";
      public static final String NAME = "limelight-notes";

      public static final double kP_AIM = 0.035;
      public static final double kP_RANGE = 0.06;
    }
    
    public static final class AprilTagConstants {
      public static final String TABLE_KEY = "limelight-apriltags";
      public static final String NAME = "limelight-apriltags";
      
      // TODO: tune + crosshair + pipeline
      public static final double kP_AIM = 0.015;
      public static final double kP_RANGE = 0.06;

      public static final double APRILTAG_HEIGHT = 53.88; // inches
      public static final double SPEAKER_HEIGHT = 84.63; // inches
      public static final double LIMELIGHT_HEIGHT = 26.461; // inches
      public static final double LIMELIGHT_ANGLE = 65; // degrees from horizontal
    }
  }
 
  public static class OperatorConstants {
    public static final double X_DEADBAND = 0.0425;
    public static final double Y_DEADBAND = 0.0425;
    public static final double Z_DEADBAND = 0.075;
    public static final double TURN_CONSTANT = 6;
  }
}
