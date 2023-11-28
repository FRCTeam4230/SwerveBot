package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  //Creating swerve modules
  private final SwerveModule frontLeft = new SwerveModule(
          Constants.DriveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
          Constants.DriveConstants.FRONT_LEFT_TURN_MOTOR_ID,
          Constants.DriveConstants.FRONT_LEFT_DRIVE_REVERSED,
          Constants.DriveConstants.FRONT_LEFT_TURN_REVERSED,
          Constants.DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD,
          Constants.DriveConstants.FRONT_LEFT_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule frontRight = new SwerveModule(
          Constants.DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
          Constants.DriveConstants.FRONT_RIGHT_TURN_MOTOR_ID,
          Constants.DriveConstants.FRONT_RIGHT_DRIVE_REVERSED,
          Constants.DriveConstants.FRONT_RIGHT_TURN_REVERSED,
          Constants.DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD,
          Constants.DriveConstants.FRONT_RIGHT_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule backLeft = new SwerveModule(
          Constants.DriveConstants.BACK_LEFT_DRIVE_MOTOR_ID,
          Constants.DriveConstants.BACK_LEFT_TURN_MOTOR_ID,
          Constants.DriveConstants.BACK_LEFT_DRIVE_REVERSED,
          Constants.DriveConstants.BACK_LEFT_TURN_REVERSED,
          Constants.DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_OFFSET_RAD,
          Constants.DriveConstants.BACK_LEFT_ABSOLUTE_ENCODER_REVERSED);

  private final SwerveModule backRight = new SwerveModule(
          Constants.DriveConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
          Constants.DriveConstants.BACK_RIGHT_TURN_MOTOR_ID,
          Constants.DriveConstants.BACK_RIGHT_DRIVE_REVERSED,
          Constants.DriveConstants.BACK_RIGHT_TURN_REVERSED,
          Constants.DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET_RAD,
          Constants.DriveConstants.BACK_RIGHT_ABSOLUTE_ENCODER_REVERSED);

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  //Constructor
  public SwerveSubsystem() {
    //The tutorial said this was needed to zero heading during startup
    //Doesn't really make sense since I thought the gyro zeroed heading automatically when turned on
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception ignored) {
      }
    }).start();

    SmartDashboard.putData(this);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void zeroHeading() {
    navx.reset();
  }

  //Returns heading between -360 to 360
  public double getHeading() {
    return -Math.IEEEremainder(navx.getAngle(), 360);
  }

  //Gets heading but as Rotation2d object
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    //Decreases speed proportionally so that none goes past the velocity limit
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
            Constants.DriveConstants.MAX_SPEED_METERS_PER_SEC);

    //Setting swerve module states
    backLeft.setDesiredState(desiredStates[0]);
    frontLeft.setDesiredState(desiredStates[1]);
    frontRight.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Front left turn encoder", frontLeft::getTurnPosition, null);
    builder.addDoubleProperty("Front right turn encoder", frontRight::getTurnPosition, null);
    builder.addDoubleProperty("Back left turn encoder", backLeft::getTurnPosition, null);
    builder.addDoubleProperty("Back right turn encoder", backRight::getTurnPosition, null);

    builder.addDoubleProperty("Front left absolute encoder", frontLeft::getAbsoluteEncoderRad, null);
    builder.addDoubleProperty("Front right absolute encoder", frontRight::getAbsoluteEncoderRad, null);
    builder.addDoubleProperty("Back left absolute encoder", backLeft::getAbsoluteEncoderRad, null);
    builder.addDoubleProperty("Back right absolute encoder", backRight::getAbsoluteEncoderRad, null);

    builder.addDoubleProperty("Front left drive encoder", frontLeft::getDrivePosition, null);
    builder.addDoubleProperty("Front right drive encoder", frontRight::getDrivePosition, null);
    builder.addDoubleProperty("Back left drive encoder", backLeft::getDrivePosition, null);
    builder.addDoubleProperty("Back right drive encoder", backRight::getDrivePosition, null);

    builder.addDoubleProperty("Front left drive motor temp celsius", frontLeft::getDriveMotorTempCelsius, null);
    builder.addDoubleProperty("Front left turn motor temp celsius", frontLeft::getTurnMotorTempCelsius, null);
    builder.addDoubleProperty("Front right drive motor temp celsius", frontRight::getDriveMotorTempCelsius, null);
    builder.addDoubleProperty("Front right turn motor temp celsius", frontRight::getTurnMotorTempCelsius, null);
    builder.addDoubleProperty("Back left drive motor temp celsius", backLeft::getDriveMotorTempCelsius, null);
    builder.addDoubleProperty("Back left turn motor temp celsius", backLeft::getTurnMotorTempCelsius, null);
    builder.addDoubleProperty("Back right drive motor temp celsius", backRight::getDriveMotorTempCelsius, null);
    builder.addDoubleProperty("Back right turn motor temp celsius", backRight::getTurnMotorTempCelsius, null);

    builder.addDoubleProperty("Robot Heading", this::getHeading, null);
  }
}
