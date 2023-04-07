package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {

    private final TalonSRX driveMotor;
    private final TalonSRX turnMotor;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driverMotorId, int turningMotorId,
            boolean driveMotorReversed, boolean turningMotorReversed,
            double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new TalonSRX(driverMotorId);
        turnMotor = new TalonSRX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(Constants.SwerveModuleConstants.kP_TURNING, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        configMotors(driveMotor);
        configMotors(turnMotor);

        turnMotor.configFeedbackNotContinuous(true, 0);
    }

    public double getDrivePosition() {
        return driveMotor.getSensorCollection().getQuadraturePosition();
    }

    public double getTurnPosition() {
        return turnMotor.getSensorCollection().getQuadraturePosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSensorCollection().getQuadratureVelocity();
    }

    public double getTurnVelocity() {
        return turnMotor.getSensorCollection().getQuadratureVelocity();
    }

    public double getAbsoluteEncoderRad() {
        //I need to figure out how to get the angle from analog input
        double angle = turnMotor.getSensorCollection().getAnalogIn();
        // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getAnalogEncoder() {
        return turnMotor.getSensorCollection().getAnalogIn();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        //Ignores small inputs
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        
        state = SwerveModuleState.optimize(state, getState().angle);

        //Setting motors
        driveMotor.set(TalonSRXControlMode.PercentOutput, 
        state.speedMetersPerSecond / Constants.DriveConstants.MAX_SPEED_METERS_PER_SEC);
        turnMotor.set(TalonSRXControlMode.PercentOutput, 
        turningPidController.calculate(getTurnPosition(), 
        state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(TalonSRXControlMode.PercentOutput, 0);
        turnMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    private void configMotors(TalonSRX motor) {
        motor.configFactoryDefault();
        motor.configOpenloopRamp(Constants.SwerveModuleConstants.RAMP_RATE);
    }

}
