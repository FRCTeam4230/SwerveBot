package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RevertToStartingWheelConfigCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final RevertToStartingWheelConfigCommand revertToStartingWheelConfigCommand = new RevertToStartingWheelConfigCommand(swerveSubsystem);
  private final XboxController driverController = new XboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  private boolean fieldOriented = true;

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    //Button A to zero heading
    new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
    new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(swerveSubsystem::resetEncoders, swerveSubsystem));
    new JoystickButton(driverController, XboxController.Button.kB.value).toggleOnTrue(new InstantCommand(() -> fieldOriented = !fieldOriented));
    new JoystickButton(driverController, XboxController.Button.kY.value).whileTrue(revertToStartingWheelConfigCommand);
  }

  private void configureDefaultCommands() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(
            swerveSubsystem,
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            () -> fieldOriented));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
