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

    private AHRS navx = new AHRS(SPI.Port.kMXP);

    //Constructor
    public SwerveSubsystem() {
        //The tutorial said this was needed to zero heading during startup
        //Doesn't really make sense since I thought the gyro zeroed heading automatically when turned on
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        SmartDashboard.putData(this);
    }

    public void zeroHeading() {
        navx.reset();
    }

    //Returns heading between -360 to 360
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    //Gets heading but as Rotation2d object
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //Decreases speed proportioanlly so that none goes past the velocity limit
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
        Constants.DriveConstants.MAX_SPEED_METERS_PER_SEC);

        //Setting swerve module states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void turnInPlace(SwerveModuleState[] desiredStates, double power) {
        //Decreases speed proportioanlly so that none goes past the velocity limit
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
        Constants.DriveConstants.MAX_SPEED_METERS_PER_SEC);

        frontLeft.turnInPlace(desiredStates[0], power);
        frontRight.turnInPlace(desiredStates[1], power);
        backLeft.turnInPlace(desiredStates[2], power);
        backRight.turnInPlace(desiredStates[3], power);

    }

    @Override
    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);

        builder.addDoubleProperty("Front Left Analog", () -> frontLeft.getAbsoluteEncoderRad(), null);
        builder.addDoubleProperty("Front Right Analog", () -> frontRight.getAbsoluteEncoderRad(), null);
        builder.addDoubleProperty("Back Left Analog", () -> backLeft.getAbsoluteEncoderRad(), null);
        builder.addDoubleProperty("Back Right Analog", () -> backRight.getAbsoluteEncoderRad(), null);

        builder.addDoubleProperty("Front left quad encoder", () -> frontLeft.getTurnPosition(), null);
        builder.addDoubleProperty("Front right quad encoder", () -> frontRight.getTurnPosition(), null);
        builder.addDoubleProperty("Back left quad encoder", () -> backLeft.getTurnPosition(), null);
        builder.addDoubleProperty("Back right quad encoder", () -> backRight.getTurnPosition(), null);

        builder.addDoubleProperty("Front left selected sensor", () -> frontLeft.getTurnMotorSelectedSensorPos(), null);
        builder.addDoubleProperty("Front right selected sensor", () -> frontRight.getTurnMotorSelectedSensorPos(), null);
        builder.addDoubleProperty("Back left selected sensor", () -> backLeft.getTurnMotorSelectedSensorPos(), null);
        builder.addDoubleProperty("Back right selected sensor", () -> backRight.getTurnMotorSelectedSensorPos(), null);
    }
}
