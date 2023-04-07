package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSupplier, ySupplier, turnSupplier;
    private final BooleanSupplier fieldOrientedSupplier;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public SwerveDriveCommand(SwerveSubsystem swerveSubsystem,
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turnSupplier,
            BooleanSupplier fieldOrientedSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        this.xLimiter = new SlewRateLimiter(Constants.DriveConstants.TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC);
        this.yLimiter = new SlewRateLimiter(Constants.DriveConstants.TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC);
        this.turnLimiter = new SlewRateLimiter(Constants.DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();
        double turnSpeed = turnSupplier.getAsDouble();

        xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.DEADBAND ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > Constants.OperatorConstants.DEADBAND ? turnSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.X_SPEED_MULTIPLIER;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.Y_SPEED_MULTIPLIER;
        turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.DriveConstants.TURN_SPEED_MULTIPLIER;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedSupplier.getAsBoolean()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        SwerveModuleState[] moduleStates = Constants.DriveConstants.KINEMATICS
        .toSwerveModuleStates(chassisSpeeds);
        
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
