package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  private final PIDController turningPidController;

  private final AbsoluteEncoder absoluteEncoder;

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

    //This might not be right
    absoluteEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    driveEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.DRIVE_ENCODER_ROT_TO_METER);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.DRIVE_ENCODER_ROT_TO_METER_PER_SEC);
    turnEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.TURN_ENCODER_ROT_TO_RAD);
    turnEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.TURN_ENCODER_RPM_TO_RAD_PER_SEC);

    turningPidController = new PIDController(Constants.SwerveModuleConstants.P_TURNING, 0, 0);
    turningPidController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

    resetEncoders();
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
    if (Math.abs(state.speedMetersPerSecond) < 0.004) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);


    double turnOutput = turningPidController.calculate(getTurnPosition(),
            state.angle.getRadians());

    // Setting motors
    driveMotor.set(MathUtil.clamp(
            state.speedMetersPerSecond * Constants.DriveConstants.DRIVE_MULTIPLIER,
            -.99, .99));

    turnMotor.set(MathUtil.clamp(turnOutput, -.99, .99));

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

}
