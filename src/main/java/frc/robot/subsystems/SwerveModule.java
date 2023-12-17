package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final SparkMaxPIDController turnPidController;
  private final SparkMaxPIDController drivePidController;

  private final DutyCycleEncoder absoluteEncoder;
  private final SimpleMotorFeedforward driveFeedforward;
  private double lastAngle;//Keeps track of the angle from previous state
  private final double absoluteEncoderOffsetRad;
  private final boolean absoluteEncoderReversed;

  public SwerveModule(Constants.SwerveModuleConstants config) {
    absoluteEncoderOffsetRad = config.getAbsoluteEncoderOffsetRad();
    absoluteEncoderReversed = config.getAbsoluteEncoderReversed();

    driveMotor = new CANSparkMax(config.getDriveMotorId(), CANSparkMaxLowLevel.MotorType.kBrushless);
    turnMotor = new CANSparkMax(config.getTurnMotorId(), CANSparkMaxLowLevel.MotorType.kBrushless);

    configMotors(driveMotor);
    configMotors(turnMotor);

    driveMotor.setInverted(config.getDriveReversed());
    turnMotor.setInverted(config.getTurnReversed());

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    absoluteEncoder = new DutyCycleEncoder(config.getAbsoluteEncoderPWMPort());

    configEncoders();

    turnPidController = turnMotor.getPIDController();
    drivePidController = driveMotor.getPIDController();

    configPidControllers();

    driveFeedforward = new SimpleMotorFeedforward(Constants.FeedForwardConstants.DRIVE_S,
            Constants.FeedForwardConstants.DRIVE_V,
            Constants.FeedForwardConstants.DRIVE_A);

    resetEncoders();

    lastAngle = getState().angle.getRadians();
  }

  // Getting encoder position
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurnPosition() {
    return turnEncoder.getPosition();
  }

  // Getting encoder velocity
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurnVelocity() {
    return turnEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoderReversed ? -absoluteEncoder.getDistance() : absoluteEncoder.getDistance();
    angle += absoluteEncoderOffsetRad;
    return angle;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public void resetWheelPosition() {
    turnPidController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public boolean wheelFacingForward() {
    //If the turn encoders are within -0.03 to 0.03, then the wheel's facing forward
    return Math.abs(turnEncoder.getPosition()) <= 0.03;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    // Ignores small inputs
    if (Math.abs(state.speedMetersPerSecond) < 0.07) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);

    drivePidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0,
            driveFeedforward.calculate((state.speedMetersPerSecond)));


    //If the input is small, don't change the angle, makes robot easier to control
    double angle =
            Math.abs(state.speedMetersPerSecond) <= Constants.DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SEC * 0.01
                    ? lastAngle
                    : state.angle.getRadians();

    //Setting the angle motor through its builtin PID controller fixed the problem of wheel
    //spins not being optimized
    turnPidController.setReference(angle, CANSparkMax.ControlType.kPosition);

    lastAngle = angle;
  }

  public double getDriveMotorTempCelsius() {
    return driveMotor.getMotorTemperature();
  }

  public double getTurnMotorTempCelsius() {
    return turnMotor.getMotorTemperature();
  }

  //Right now if a motor is over 30 degrees celsius it is considered overheating
  //The threshold should be fine-tuned after practice driving to see that regular driving gets the temp to
  public boolean isDriveMotorOverheating() {
    return getDriveMotorTempCelsius() >= 30;
  }

  public boolean isTurnMotorOverheating() {
    return getTurnMotorTempCelsius() >= 30;
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }


  private void configMotors(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    motor.setOpenLoopRampRate(Constants.DriveConstants.RAMP_RATE);
  }

  private void configEncoders() {
    driveEncoder.setPositionConversionFactor(Constants.ConversionFactors.DRIVE_ENCODER_ROT_TO_METER);
    driveEncoder.setVelocityConversionFactor(Constants.ConversionFactors.DRIVE_ENCODER_ROT_TO_METER_PER_SEC);
    turnEncoder.setPositionConversionFactor(Constants.ConversionFactors.TURN_ENCODER_ROT_TO_RAD);
    turnEncoder.setVelocityConversionFactor(Constants.ConversionFactors.TURN_ENCODER_RPM_TO_RAD_PER_SEC);

    absoluteEncoder.setDistancePerRotation(2 * Math.PI);
  }

  private void configPidControllers() {
    drivePidController.setP(Constants.PIDFConstants.TURN_P);
    drivePidController.setI(Constants.PIDFConstants.TURN_I);
    drivePidController.setD(Constants.PIDFConstants.TURN_D);
    drivePidController.setFF(Constants.PIDFConstants.TURN_F);

    turnPidController.setP(Constants.PIDFConstants.DRIVE_P);
    turnPidController.setI(Constants.PIDFConstants.DRIVE_I);
    turnPidController.setD(Constants.PIDFConstants.DRIVE_D);
    turnPidController.setFF(Constants.PIDFConstants.DRIVE_F);

    turnPidController.setPositionPIDWrappingEnabled(true);
    turnPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);
    turnPidController.setPositionPIDWrappingMinInput(0);
  }

}
