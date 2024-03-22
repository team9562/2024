// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = Units.lbsToKilograms(120.8);
  public static final Matter CHASSIS = new Matter(new Translation3d(Units.inchesToMeters(28), Units.inchesToMeters(32), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(4.4196, 4.25, 322.415, 605.58);

  public static final class MotorConstants { 
    public static final int NEO_550_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_550_FREE_LIMIT = 1;

    public static final int NEO_V1_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_V1_FREE_LIMIT = 1;

    public static final int NEO_V1_MAX_RPMS = 5600; // 5676, this is a safe limit
    public static final int NEO_550_MAX_RPMS = 10000; // 11K, this is a safe limit
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0,0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(3.375, 0.005, 0);
  }

  public static final class DrivebaseConstants {
    public static final double LOCK_TIME = 10; // seconds
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
    
    public static final int STALL_LIMIT = 65;
    
    public static final double ELEVATOR_THRESHOLD = 0.25;
    
    public static final double SAFE_MIN_PERCENT = 0.065;
    public static final double SAFE_MAX_PERCENT = 0.824;
  }
  
  public static final class ShooterConstants {
    public static final int LEFT_CAN = 15;
    public static final int RIGHT_CAN = 16;
    public static final int FEEDER_CAN = 17;
    
    public static final int LIMIT_SWITCH_PORT = 7;
    
    // TODO: Find values
    // Bottom
    public static final double kP_LEFT = 1;
    public static final double kI_LEFT = 0;
    public static final double kD_LEFT = 0;
    public static final double kFF_LEFT = 0.0006;
    
    // TODO: Find values
    // Top
    public static final double kP_RIGHT = 0.75;
    public static final double kI_RIGHT = 0;
    public static final double kD_RIGHT = 0;
    public static final double kFF_RIGHT = 0.00015;

    public static final double kP_FEEDER = 0.000125;
    public static final double kI_FEEDER = 0;
    public static final double kD_FEEDER = 0.125;
    public static final double kFF_FEEDER = 0.0001;
    
    public static final int STALL_LIMIT = 50;
    public static final int FEEDER_STALL_LIMIT = 40;
    
    public static final double SENSOR_THRESHOLD_INCHES = 0.5;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_CAN = 18;
    
    public static final int STALL_LIMIT = 35;
  }
  
  public static final class AngleConstants {
    public static final int ANGLE_CAN = 19;
    
    public static final double ANGLE_MAX_ABS = 0.558;
    public static final double ANGLE_MIN_ABS = 0.22;
    public static final double ANGLE_MIN_REL = 0;
    public static final double ANGLE_MAX_REL = 53.4;
    
    public static final int ENCODER_PORT = 8;
    
    public static final double kP = 0.03;
    public static final double kI = 0;
    public static final double kD = 0.0625;
    public static final double kFF = 0.0003;

    public static final int STALL_LIMIT = 35;

    public static final double ANGLE_THRESHOLD = 0.5;
    // public static final double ANGLE_THRESHOLD = 0.25;
  }

  public static final class VisionConstants {
    public static final String TABLE_KEY = "limelight";
    public static final String NAME = "limelight";

    public static final double kP_AIM = 0.035;
    public static final double kP_RANGE = 0.1;
  }
 
  public static class OperatorConstants {
    // Joystick Deadband
    public static final double X_DEADBAND = 0.055;
    public static final double Y_DEADBAND = 0.055;
    public static final double Z_DEADBAND = 0.075;
    public static final double TURN_CONSTANT = 6;
  }
}
