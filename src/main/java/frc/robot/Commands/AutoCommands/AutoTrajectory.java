// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.AutoCommands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Drivetrains.SwerveDrivetrain;

public class AutoTrajectory extends Command {
  /** Creates a new AutoPathPathPlanner. */
  private PathPlannerTrajectory  trajectory;
  private Timer timer = new Timer();

  private boolean hasReachedEndOfTrajectory = false;

  private SwerveDrivetrain drivetrain;
  public AutoTrajectory(SwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    // PathPlannerPath tmp = PathPlannerPath.fromPathFile("StraightPath");
    // trajectory = tmp.getTrajectory(
    //   new ChassisSpeeds(0, 0, 0), 
    //     Rotation2d.fromRadians(0));

    // Transform2d offset = trajectory.getInitialTargetHolonomicPose().minus(drivetrain.getOdometry().getPoseMeters());

    // trajectory.getInitialTargetHolonomicPose().transformBy(offset);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Pose2d startingPos = trajectory.getInitialTargetHolonomicPose();

    drivetrain.resetModuleHeadings();
    drivetrain.resetOdometry(startingPos);
    

    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // PathPlannerTrajectory.State state = (PathPlannerTrajectory.State) trajectory.sample(timer.get());
    
    // drivetrain.updateSkew(false);
    // ChassisSpeeds chassisSpeeds = SwerveConstants.SWERVE_PID_CONTROLLER.calculate(
    //   drivetrain.getOdometry().getPoseMeters(),  
    //   state.getTargetHolonomicPose(), 
    //   state.velocityMps, 
    //   state.targetHolonomicRotation);   

    // SmartDashboard.putNumber("turnspeed", chassisSpeeds.omegaRadiansPerSecond);
    // SmartDashboard.putNumber("target holo heading:", state.targetHolonomicRotation.getDegrees());
    // SmartDashboard.putNumber("drivetrain heading: ", drivetrain.getPigeonRotation2dEM().getDegrees());

    // drivetrain.setCentralMotion(chassisSpeeds, null);

    // hasReachedEndOfTrajectory = state.equals(trajectory.getEndState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.updateSkew(false);
    //drivetrain.reface(drivetrain.getPigeonRotation2dEM().getDegrees(), trajectory.getEndState().targetHolonomicRotation.getDegrees());
    timer.stop();
  }

  // Returns true when the command should end.
  @Override 
  public boolean isFinished() {
    return false;
    //return timer.get()-trajectory.getTotalTimeSeconds()>=.25 && hasReachedEndOfTrajectory;
  }
}
