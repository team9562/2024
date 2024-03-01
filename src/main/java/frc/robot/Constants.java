// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static final double ROBOT_MASS = Units.lbsToKilograms(110);
  // public static final Matter CHASSIS = new Matter(new Translation3d(Units.inchesToMeters(28), Units.inchesToMeters(32), Units.inchesToMeters(8)), ROBOT_MASS);
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, 0), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  
  public static final class MotorConstants { 
    public static final int NEO_550_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_550_STALL_LIMIT = 60;
    public static final int NEO_550_FREE_LIMIT = 1;

    public static final int NEO_V1_NOMINAL_VOLTAGE = 12;
    
    public static final int NEO_V1_STALL_LIMIT_LOW = 30;
    public static final int NEO_V1_STALL_LIMIT_HIGH = 60;
    public static final int NEO_V1_FREE_LIMIT = 1;

    public static final int NEO_V1_MAX_RPMS = 5600; // 5676, this is a safe limit
    public static final int NEO_550_MAX_RPMS = 10000; // 11K, this is a safe limit
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants {
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ElevatorConstants {
    public static final double SPROCKET_RADIUS = 4.5;
    public static final double GEAR_RATIO = 16;

    public static final int LEFT_CAN = 13;
    public static final int RIGHT_CAN = 14;

    public static final int LIMIT_SWITCH_PORT = 9;
    
    public static final double MAX_HEIGHT = (0 / GEAR_RATIO) * (2 * Math.PI * SPROCKET_RADIUS); // TODO: Homing sequence, then find max
    public static final double MIN_HEIGHT = 0;

    // TODO: Find values
    public static final double kP = 0.004;
    public static final double kI = 0;
    public static final double kD = 1.5;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
  }

  public static final class ShooterConstants {
    public static final int LEFT_CAN = 15;
    public static final int RIGHT_CAN = 16;
    public static final int FEEDER_CAN = 17;

    // TODO: Find values
    public static final double kP = 0.004;
    public static final double kI = 0;
    public static final double kD = 1.5;
    public static final double kFF = 0.0001;

    public static final double SENSOR_THRESHOLD_INCHES = 0.5;
  }
  
  public static final class IntakeConstants {
    public static final int INTAKE_CAN = 18;
  }
  
  public static final class AngleConstants {
    public static final int ANGLE_CAN = 19;

    public static final double ANGLE_MAX = 50;
    public static final double ANGLE_MIN = -40;

    public static final int ENCODER_PORT = 8;
    
    // TODO: Find values
    public static final double kP = 0.004;
    public static final double kI = 0;
    public static final double kD = 1.5;
    public static final double kFF = 0.0001;

    public static final double ANGLE_THRESHOLD_DEGREES = 5;
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }
}
