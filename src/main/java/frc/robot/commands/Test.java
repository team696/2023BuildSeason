// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Test extends CommandBase {
  Swerve swerve;
SwerveControllerCommand swerveControllerCommand;
  /** Creates a new Test. */
  public Test(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  
    TrajectoryConfig config =
        new TrajectoryConfig(
                0.5,
                0.5)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
    
    double[] pose = Constants.AutoConstants.RobotPositions[swerve.tag][swerve.height][swerve.hor];

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(List.of(
            new Pose2d(pose[0]+0.7, pose[1], new Rotation2d(Math.PI)),
            new Pose2d(pose[0], pose[1], new Rotation2d(Math.PI))),
        config);
    
    var thetaController =
        new ProfiledPIDController(
            0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

     swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

    swerve.m_fieldSim.getObject("traj2").setTrajectory(exampleTrajectory);
  
  swerve.normalizeOdometry();
}

    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveControllerCommand.schedule();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
