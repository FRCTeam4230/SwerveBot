package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class YAGSLSubsystem extends SubsystemBase {

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive swerveDrive  = new SwerveParser(swerveJsonDirectory).createSwerveDrive();


  public YAGSLSubsystem() {}

  @Override
  public void periodic() {
  }
}
