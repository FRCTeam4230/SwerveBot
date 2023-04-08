// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final XboxController driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    //Button A to zero heading
    new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
  }

  private void configureDefaultCommands() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveCommand(
        swerveSubsystem,
        () -> driverController.getLeftY(),
        () -> driverController.getLeftX(),
        () -> driverController.getRightX(),
        () -> true));
  }

  public Command getAutonomousCommand() {
    return null;
}}
