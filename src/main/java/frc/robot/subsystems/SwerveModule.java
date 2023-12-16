package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final SparkMaxPIDController turnPidController;
  private final SparkMaxPIDController drivePidController;

  private final AbsoluteEncoder absoluteEncoder;
  private final SimpleMotorFeedforward driveFeedforward;
  private double lastAngle;//Keeps track of the angle from previous state
  private double absoluteEncoderOffsetRad;
  private boolean absoluteEncoderReversed;

  public SwerveModule(int driverMotorId, int turningMotorId,
                      boolean driveMotorReversed, boolean turningMotorReversed,
                      double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {
    this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
    this.absoluteEncoderReversed = absoluteEncoderReversed;

    driveMotor = new CANSparkMax(driverMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    turnMotor = new CANSparkMax(turningMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    configMotors(driveMotor);
    configMotors(turnMotor);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    configEncoders();

    //This might not be right
    absoluteEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);


    turnPidController = turnMotor.getPIDController();
    drivePidController = driveMotor.getPIDController();

    configPidControllers();

    driveFeedforward = new SimpleMotorFeedforward(Constants.SwerveModuleConstants.FeedForwardConstants.DRIVE_S,
            Constants.SwerveModuleConstants.FeedForwardConstants.DRIVE_V,
            Constants.SwerveModuleConstants.FeedForwardConstants.DRIVE_A);

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
    return absoluteEncoder.getPosition();
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(0);
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

    //Setting the angle motor through its builtin PID controller seems to fix the problem of wheel
    //spins not being optimized, maybe it takes too much time for the signal from integrated encoders to reach the roborio
    //and get sent back for the spins to be accurate, and by using Spark Max's builtin computer we sped things up
    turnPidController.setReference(angle, CANSparkMax.ControlType.kPosition);

    lastAngle = angle;
  }

  public CANSparkMax getTurnMotor() {
    return turnMotor;
  }

  public double getDriveMotorTempCelsius() {
    return driveMotor.getMotorTemperature();
  }

  public double getTurnMotorTempCelsius() {
    return turnMotor.getMotorTemperature();
  }

  //Right now if a motor is over 30 degrees celsius it is considered overheating
  //The threshold should be finetuned after practice driving to see that regular driving gets the temp to
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
    motor.setOpenLoopRampRate(Constants.SwerveModuleConstants.RAMP_RATE);
  }

  private void configEncoders() {
    driveEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.ConversionFactors.DRIVE_ENCODER_ROT_TO_METER);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.ConversionFactors.DRIVE_ENCODER_ROT_TO_METER_PER_SEC);
    turnEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.ConversionFactors.TURN_ENCODER_ROT_TO_RAD);
    turnEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.ConversionFactors.TURN_ENCODER_RPM_TO_RAD_PER_SEC);
  }

  private void configPidControllers() {
    drivePidController.setP(Constants.SwerveModuleConstants.PIDFConstants.TURN_P);
    drivePidController.setI(Constants.SwerveModuleConstants.PIDFConstants.TURN_I);
    drivePidController.setD(Constants.SwerveModuleConstants.PIDFConstants.TURN_D);
    drivePidController.setFF(Constants.SwerveModuleConstants.PIDFConstants.TURN_F);

    turnPidController.setP(Constants.SwerveModuleConstants.PIDFConstants.DRIVE_P);
    turnPidController.setI(Constants.SwerveModuleConstants.PIDFConstants.DRIVE_I);
    turnPidController.setD(Constants.SwerveModuleConstants.PIDFConstants.DRIVE_D);
    turnPidController.setFF(Constants.SwerveModuleConstants.PIDFConstants.DRIVE_F);

    turnPidController.setPositionPIDWrappingEnabled(true);
    turnPidController.setPositionPIDWrappingMaxInput(2 * Math.PI);
    turnPidController.setPositionPIDWrappingMinInput(0);
  }

}
