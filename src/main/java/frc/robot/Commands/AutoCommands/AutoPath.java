// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;


public class AutoPath {
  /** Creates a new AutoPath. */
  private SwerveDrivetrain drivetrain;
  // private PathPlannerTrajectory trajectory = PathPlannerPath.fromPathFile("StraightPath").
  //       getTrajectory(new ChassisSpeeds(0, 0, 0), 
  //       Rotation2d.fromRadians(0));
  
  public SwerveControllerCommand autoCommand;


  //Library no longer inherits from WPILIB trajectory; this constructor broke

  public AutoPath(SwerveDrivetrain drivetrain) {
    // this.drivetrain = drivetrain;
    // autoCommand =  new SwerveControllerCommand(
    //   trajectory, 
    //   () -> drivetrain.getOdometry().getPoseMeters(), 
    //   drivetrain.getKinematics(), 
    //   SWERVE_PID_CONTROLLER, 
    //   drivetrain::setStates, drivetrain);
  }
}
