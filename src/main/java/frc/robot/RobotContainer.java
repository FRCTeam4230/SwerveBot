package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final XboxController driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

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
    }

    private void configureDefaultCommands() {
        swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(
                swerveSubsystem,
                //The robot thinks that x means front and back, y means left and right
                //That's the opposite of what we think when we use joysticks
                //So X is being passed in as Y, and Y is being passed as X to fix that
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                //Eventually, make this a toggle button that allows the user to switch between
                //field centric and robot centric driving
                () -> fieldOriented));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
