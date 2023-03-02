// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import java.util.List;

// package edu.wpi.first.math.trajectory.constraint;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.commands.AutoBalanceStation;
import frc.robot.commands.AutoLockToGamePiece;
import frc.robot.commands.AutoSwervePositions;
import frc.robot.commands.BalanceStation;
import frc.robot.commands.GripperStateCommand;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.HoldArmPos;
import frc.robot.commands.Test;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Gripper.GripperState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class BalanceAuto extends SequentialCommandGroup {
  Swerve swerve;

  SwerveControllerCommand swerveControllerCommand1;
  SwerveControllerCommand swerveControllerCommand2;
  SwerveControllerCommand swerveControllerCommand3;
  SwerveControllerCommand swerveControllerCommand4;
  SwerveControllerCommand swerveControllerCommand5;
  SwerveControllerCommand swerveControllerCommand6;


  /** Creates a new AutoPlaceTest. */
  public BalanceAuto(Swerve swerve) {
    this.swerve = swerve;
  
 
    
           
    // constraints.getMinMaxAccelerationMetersPerSecondSq(null, 0, 0)
    TrajectoryConfig config = new TrajectoryConfig(
        0.5,
        0.3)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
        config.addConstraint(new CentripetalAccelerationConstraint(0.3));
    TrajectoryConfig configrev = new TrajectoryConfig(
        0.5,
        0.3)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
        configrev.addConstraint(new CentripetalAccelerationConstraint(0.3));


        Trajectory traj1 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(1.97, 2.584, new Rotation2d(Math.PI)),
            new Pose2d(3.85, 2.584, new Rotation2d(Math.PI))),
        configrev);

        Trajectory traj2 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(3.85, 2.584, new Rotation2d(Math.PI)),
            new Pose2d(5.4, 2.584, new Rotation2d(Math.PI))),
        configrev);
        Trajectory traj3 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(5.4, 2.584, new Rotation2d(Math.PI)),
            new Pose2d(3.6, 2.584, new Rotation2d(Math.PI))),
        config);


       
      
          


    var thetaController = new ProfiledPIDController(
        1, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand1 = new SwerveControllerCommand(
        traj1,
        swerve::getAprilTagEstPosition,
        Constants.Swerve.swerveKinematics,
        new PIDController(0.5, 0, 0),
        new PIDController(0.5, 0, 0),
        thetaController,
        swerve::setModuleStates,
        swerve);
        swerveControllerCommand2 = new SwerveControllerCommand(
          traj2,
          swerve::getAprilTagEstPosition,
          Constants.Swerve.swerveKinematics,
          new PIDController(0.5, 0, 0),
          new PIDController(0.5, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve);
  
          swerveControllerCommand3 = new SwerveControllerCommand(
            traj3,
            swerve::getAprilTagEstPosition,
            Constants.Swerve.swerveKinematics,
            new PIDController(0.5, 0, 0),
            new PIDController(0.5, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
    
         
    addCommands(
      
      swerveControllerCommand1,
      swerveControllerCommand2,
      swerveControllerCommand3,
      new AutoBalanceStation(swerve,  true, false).deadlineWith(new WaitCommand(10))
     


      
      );


    
  }
}
