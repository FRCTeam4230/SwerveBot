// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class SwerveModuleConstants {
    public static final double kP_TURNING = 0;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 0;
    public static final double TURN_MOTOR_GEAR_RATIO = 0;
    public static final double WHEEL_DIAMETER_METERS = 0;
    public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    public static final double TURN_ENCODER_ROT_TO_RAD = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double DRIVE_ENCODER_RPM_TO_METER_PER_SEC = DRIVE_ENCODER_ROT_TO_METER / 60;
    public static final double TURN_ENCODER_RPM_TO_RAD_PER_SEC = TURN_ENCODER_ROT_TO_RAD / 60;
    public static final double RAMP_RATE = 0;
  }

  public static class DriveConstants {

    public static final double MAX_SPEED_METERS_PER_SEC = 0;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 0;
    public static final boolean FRONT_LEFT_DRIVE_REVERSED = false;
    public static final boolean FRONT_LEFT_TURN_REVERSED = false;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_PORT = 0;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 0;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 0;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSED = false;
    public static final boolean FRONT_RIGHT_TURN_REVERSED = false;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_PORT = 0;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 0;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 0;
    public static final boolean BACK_LEFT_DRIVE_REVERSED = false;
    public static final boolean BACK_LEFT_TURN_REVERSED = false;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_PORT = 0;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 0;
    public static final boolean BACK_RIGHT_DRIVE_REVERSED = false;
    public static final boolean BACK_RIGHT_TURN_REVERSED = false;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_PORT = 0;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC = 0;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC = 0;

    public static final double X_SPEED_MULTIPLIER = 0;
    public static final double Y_SPEED_MULTIPLIER = 0;
    public static final double TURN_SPEED_MULTIPLIER = 0;

    public static final double TRACK_WIDTH = Units.inchesToMeters(0); //Distance between left and right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(0); //Distance between front and back wheels

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), 
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)
    );
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0;
  }
}
