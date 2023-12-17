package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class PIDFConstants {
    public static final double DRIVE_P = 0.3;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_F = 0.0;

    public static final double TURN_P = 0.1;
    public static final double TURN_D = 0.2;
    public static final double TURN_I = 0;
    public static final double TURN_F = 0.0;
  }

  public static class FeedForwardConstants {
    public static final double DRIVE_S = 0.11;
    public static final double DRIVE_V = 1.0;//2.6335
    public static final double DRIVE_A = 0;
  }

  public enum SwerveModuleConstants {
    FRONT_LEFT(8, 7, false, false,
            1, false, 1),
    FRONT_RIGHT(2, 1, true, false,
            0.312, true, 0),
    BACK_LEFT(6, 5, false, false,
            2, false, 2),
    BACK_RIGHT(4, 3, false, false,
            0, false, 3);
    private final int driveMotorId;
    private final int turnMotorId;
    private final boolean driveReversed;
    private final boolean turnReversed;
    private final double absoluteEncoderOffsetRad;
    private final boolean absoluteEncoderReversed;
    private final int absoluteEncoderPWMPort;

    SwerveModuleConstants(
            int driveMotorId, int turnMotorId, boolean driveReversed, boolean turnReversed,
            double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed, int absoluteEncoderPWMPort
    ) {
      this.driveMotorId = driveMotorId;
      this.turnMotorId = turnMotorId;
      this.driveReversed = driveReversed;
      this.turnReversed = turnReversed;
      this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      this.absoluteEncoderPWMPort = absoluteEncoderPWMPort;
    }

    public int getDriveMotorId() {
      return driveMotorId;
    }

    public int getTurnMotorId() {
      return turnMotorId;
    }

    public boolean getDriveReversed() {
      return driveReversed;
    }

    public boolean getTurnReversed() {
      return turnReversed;
    }

    public double getAbsoluteEncoderOffsetRad() {
      return absoluteEncoderOffsetRad;
    }

    public boolean getAbsoluteEncoderReversed() {
      return absoluteEncoderReversed;
    }

    public int getAbsoluteEncoderPWMPort() {
      return absoluteEncoderPWMPort;
    }
  }

  public static class ConversionFactors {
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 1.0 / 8.14;
    public static final double TURN_MOTOR_GEAR_RATIO = 7.0 / 150.0;
    public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_MOTOR_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_ENCODER_ROT_TO_METER_PER_SEC = DRIVE_ENCODER_ROT_TO_METER / 60.0;
    public static final double TURN_ENCODER_ROT_TO_RAD = TURN_MOTOR_GEAR_RATIO * 2.0 * Math.PI;
    public static final double TURN_ENCODER_RPM_TO_RAD_PER_SEC = TURN_ENCODER_ROT_TO_RAD / 60.0;
    public static final double ABSOLUTE_ENCODER_ROT_TO_RAD = 0;
    public static final double ABSOLUTE_ENCODER_RPM_TO_RAD_PER_SEC = ABSOLUTE_ENCODER_ROT_TO_RAD / 60.0;
  }

  public static class DriveConstants {
    public static final double RAMP_RATE = 0.2;
    public static final double DRIVE_MAX_METERS_PER_SECOND = 1.5;
    public static final double MAX_RADIANS_PER_SECOND = 5.0;
    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SEC = 4.6025;
    public static final double TRACK_WIDTH = Units.inchesToMeters(10.75); //Distance between left and right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(10.75); //Distance between front and back wheels

    public static final Translation2d FRONT_LEFT_TRANSLATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FRONT_RIGHT_TRANSLATION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BACK_LEFT_TRANSLATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d BACK_RIGHT_TRANSLATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT_TRANSLATION,
            FRONT_RIGHT_TRANSLATION,
            BACK_LEFT_TRANSLATION,
            BACK_RIGHT_TRANSLATION
    );
  }

  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double DEADBAND = 0.1;
  }
}
