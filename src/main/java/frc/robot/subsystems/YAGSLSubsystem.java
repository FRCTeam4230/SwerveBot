package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class YAGSLSubsystem extends SubsystemBase {

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  public YAGSLSubsystem() {
    
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive();
    }
    catch(IOException ioe){
      //do whatever with exception
      ioe.printStackTrace();
      throw new RuntimeException(ioe);
    }
  }

  @Override
  public void periodic() {
  }
}
