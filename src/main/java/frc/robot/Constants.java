package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class SwerveModuleConstants {
    //Needs to be tuned
    public static final double P_TURNING = 0.3;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1.0 / 8.14;
    public static final double TURN_MOTOR_GEAR_RATIO = 7.0 / 150.0;
    public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    public static final double DRIVE_ENCODER_ROT_TO_METER_PER_SEC = DRIVE_ENCODER_ROT_TO_METER / 60;
    public static final double TURN_ENCODER_ROT_TO_RAD = TURN_MOTOR_GEAR_RATIO * 2 * Math.PI;
    public static final double TURN_ENCODER_RPM_TO_RAD_PER_SEC = TURN_ENCODER_ROT_TO_RAD / 60;
    public static final double RAMP_RATE = 0.2;
    public static final double ABSOLUTE_ENCODER_ROT_TO_RAD = 0;
    public static final double ABSOLUTE_ENCODER_RPM_TO_RAD_PER_SEC = ABSOLUTE_ENCODER_ROT_TO_RAD / 60;
  }

  public static class DriveConstants {

    public static final double MAX_SPEED_METERS_PER_SEC = 0.3;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
    public static final int FRONT_LEFT_TURN_MOTOR_ID = 7;
    public static final boolean FRONT_LEFT_DRIVE_REVERSED = false;
    public static final boolean FRONT_LEFT_TURN_REVERSED = false;
    public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int FRONT_RIGHT_TURN_MOTOR_ID = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
    public static final boolean FRONT_RIGHT_DRIVE_REVERSED = true;
    public static final boolean FRONT_RIGHT_TURN_REVERSED = false;
    public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 6;
    public static final int BACK_LEFT_TURN_MOTOR_ID = 5;
    public static final boolean BACK_LEFT_DRIVE_REVERSED = false;
    public static final boolean BACK_LEFT_TURN_REVERSED = false;
    public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean BACK_LEFT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 4;
    public static final int BACK_RIGHT_TURN_MOTOR_ID = 3;
    public static final boolean BACK_RIGHT_DRIVE_REVERSED = true;
    public static final boolean BACK_RIGHT_TURN_REVERSED = false;
    public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD = 0;
    public static final boolean BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double X_SPEED_MULTIPLIER = 3;
    public static final double Y_SPEED_MULTIPLIER = X_SPEED_MULTIPLIER;
    public static final double TURN_SPEED_MULTIPLIER = 3;

    public static final double TRACK_WIDTH = Units.inchesToMeters(10.75); //Distance between left and right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(10.75); //Distance between front and back wheels

    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION
    );
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.04;
  }
}
