package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final DoubleSupplier xSupplier, ySupplier, turnSupplier;
  private final BooleanSupplier fieldOrientedSupplier;

  public SwerveDriveCommand(SwerveSubsystem swerveSubsystem,
                            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier turnSupplier,
                            BooleanSupplier fieldOrientedSupplier) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
    this.fieldOrientedSupplier = fieldOrientedSupplier;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // Updating numbers
    //xSpeed is negated because the input from the joysticks for the horizontal axis is opposite of what
    //is expected
    double xSpeed = -xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();
    double turnSpeed = turnSupplier.getAsDouble();

    // Applying a deadband manually
    xSpeed = Math.abs(xSpeed) > Constants.OperatorConstants.DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.OperatorConstants.DEADBAND ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > Constants.OperatorConstants.DEADBAND ? turnSpeed : 0.0;

    // Limiting acceleration
    xSpeed *= Constants.DriveConstants.X_MAX_METERS_PER_SECOND;
    ySpeed *= Constants.DriveConstants.Y_MAX_METERS_PER_SECOND;
    turnSpeed *= Constants.DriveConstants.MAX_RADIANS_PER_SECOND;

    // Creates chassis speeds object
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedSupplier.getAsBoolean()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turnSpeed, swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    // Uses chassis speeds object and kinematics object to calculate swerve module
    // states
    SwerveModuleState[] moduleStates = Constants.DriveConstants.KINEMATICS
            .toSwerveModuleStates(chassisSpeeds);

    // Sets speed and rotation
    swerveSubsystem.setModuleStates(moduleStates);

    SmartDashboard.putBoolean("field orientation", fieldOrientedSupplier.getAsBoolean());
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
