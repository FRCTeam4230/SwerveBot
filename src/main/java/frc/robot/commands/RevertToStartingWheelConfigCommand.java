package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class RevertToStartingWheelConfigCommand extends CommandBase {
  SwerveSubsystem swerveSubsystem;

  public RevertToStartingWheelConfigCommand(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    swerveSubsystem.resetWheelPositions();
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return swerveSubsystem.wheelsFacingForward();
  }

  @Override
  public void end(boolean interrupted) {

  }
}
