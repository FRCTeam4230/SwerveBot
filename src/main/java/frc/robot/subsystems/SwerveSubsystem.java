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
  private final SwerveModule frontLeft = new SwerveModule(Constants.SwerveModuleConstants.FRONT_LEFT);

  private final SwerveModule frontRight = new SwerveModule(Constants.SwerveModuleConstants.FRONT_RIGHT);

  private final SwerveModule backLeft = new SwerveModule(Constants.SwerveModuleConstants.BACK_LEFT);

  private final SwerveModule backRight = new SwerveModule(Constants.SwerveModuleConstants.BACK_RIGHT);

  private final AHRS navx = new AHRS(SPI.Port.kMXP);

  //Constructor
  public SwerveSubsystem() {
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

  //Returns heading between 0 to 360
  public double getHeading() {
    double heading = Math.IEEEremainder(navx.getAngle(), 360);
    if (heading < 0) {
      heading += 360.0;
    }

    return heading;
  }

  public double getRawHeading() {
    return navx.getAngle();
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

  public boolean wheelsFacingForward() {
    return frontLeft.wheelFacingForward()
            && frontRight.wheelFacingForward()
            && backLeft.wheelFacingForward()
            && backRight.wheelFacingForward();
  }

  public void resetWheelPositions() {
    frontLeft.resetWheelPosition();
    frontRight.resetWheelPosition();
    backLeft.resetWheelPosition();
    backRight.resetWheelPosition();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    //Decreases speed proportionally so that none goes past the velocity limit
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
            Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SEC);

    //Setting swerve module states
    frontLeft.setDesiredState(desiredStates[1]);
    frontRight.setDesiredState(desiredStates[0]);
    backLeft.setDesiredState(desiredStates[3]);
    backRight.setDesiredState(desiredStates[2]);

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
    builder.addDoubleProperty("Robot Raw Angle", this::getRawHeading, null);
  }
}
