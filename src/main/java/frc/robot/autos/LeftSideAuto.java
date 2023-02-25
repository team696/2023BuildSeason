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
import frc.robot.commands.AutoLockToGamePiece;
import frc.robot.commands.AutoSwervePositions;
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

public class LeftSideAuto extends SequentialCommandGroup {
  Swerve swerve;
  ArmSub armSub;
  Gripper gripper;
  SwerveControllerCommand swerveControllerCommand1;
  SwerveControllerCommand swerveControllerCommand2;
  SwerveControllerCommand swerveControllerCommand3;
  SwerveControllerCommand swerveControllerCommand4;
  SwerveControllerCommand swerveControllerCommand5;
  SwerveControllerCommand swerveControllerCommand6;

MinMax minMax;
    TrajectoryConstraint constraints;

  /** Creates a new AutoPlaceTest. */
  public LeftSideAuto(Swerve swerve, ArmSub armSub, Gripper gripper) {
    this.swerve = swerve;
    this.armSub = armSub;
    this.gripper = gripper;
 
    
           
    // constraints.getMinMaxAccelerationMetersPerSecondSq(null, 0, 0)
    TrajectoryConfig config = new TrajectoryConfig(
        1.3,
        1.3)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
        config.addConstraint(new CentripetalAccelerationConstraint(0.3));
    TrajectoryConfig configrev = new TrajectoryConfig(
        1.3,
        1.3)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
        configrev.addConstraint(new CentripetalAccelerationConstraint(0.3));
        Trajectory traj1 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(3, 5.06, new Rotation2d(Math.PI)),
            new Pose2d(2.2, 5.15, new Rotation2d(Math.PI))),
        config);

        Trajectory traj2 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(2.1, 5.15, new Rotation2d(Math.PI)),
            new Pose2d(2.8, 5.15, new Rotation2d(Math.PI))),
        configrev);

        Trajectory traj3 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(2.8, 5.06, new Rotation2d(Math.PI)),
            new Pose2d(4.5, 4.5, new Rotation2d(0))),
        configrev);

        Trajectory traj4 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(4.5, 4.5, new Rotation2d(Math.PI)),
            new Pose2d(2.8, 5.06, new Rotation2d(Math.PI))),
        configrev);

        Trajectory traj5 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(3, 5.06, new Rotation2d(Math.PI)),
            new Pose2d(1.65, 5.15, new Rotation2d(Math.PI))),
        config);
        Trajectory traj6 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(2.1, 5.15, new Rotation2d(Math.PI)),
            new Pose2d(2.8, 5.15, new Rotation2d(Math.PI))),
        configrev);

      
          


    var thetaController = new ProfiledPIDController(
        1, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand1 = new SwerveControllerCommand(
        traj1,
        swerve::getAprilTagEstPosition,
        Constants.Swerve.swerveKinematics,
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController,
        swerve::setModuleStates,
        swerve);

        swerveControllerCommand2 = new SwerveControllerCommand(
          traj2,
          swerve::getAprilTagEstPosition,
          Constants.Swerve.swerveKinematics,
          new PIDController(1, 0, 0),
          new PIDController(1, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve);

          swerveControllerCommand3 = new SwerveControllerCommand(
            traj3,
            swerve::getAprilTagEstPosition,
            Constants.Swerve.swerveKinematics,
            new PIDController(0.7, 0, 0),
            new PIDController(0.7, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
            swerveControllerCommand4 = new SwerveControllerCommand(
              traj4,
              swerve::getAprilTagEstPosition,
              Constants.Swerve.swerveKinematics,
              new PIDController(0.7, 0, 0),
              new PIDController(0.7, 0, 0),
              thetaController,
              swerve::setModuleStates,
              swerve);
              swerve.m_fieldSim.getObject("traj2").setTrajectory(traj1);
              swerve.m_fieldSim.getObject("traj2").setTrajectory(traj2);
              swerve.m_fieldSim.getObject("traj2").setTrajectory(traj3);
              swerve.m_fieldSim.getObject("traj2").setTrajectory(traj4);

              swerveControllerCommand5 = new SwerveControllerCommand(
                traj5,
                swerve::getAprilTagEstPosition,
                Constants.Swerve.swerveKinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);

                swerveControllerCommand6 = new SwerveControllerCommand(
                traj6,
                swerve::getAprilTagEstPosition,
                Constants.Swerve.swerveKinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
                
    addCommands(
      
      new WaitCommand(0.5).raceWith(new HoldArmPos(armSub, ArmPositions.MID_SCORE_CONE)),
      swerveControllerCommand1.raceWith(new HoldArmPos(armSub, ArmPositions.MID_SCORE_CONE)),
      new GripperStateCommand(gripper, GripperState.OPEN),

      swerveControllerCommand2.raceWith(new WaitCommand(0.5).andThen(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP))),
      swerveControllerCommand3.raceWith(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP)),
      new AutoLockToGamePiece(swerve, 0, 0, false, false , 0).raceWith(new GroundIntake(armSub, gripper)),
      swerveControllerCommand4.raceWith(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP)),
       new WaitCommand(0.5).raceWith(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_CONE)),
      swerveControllerCommand5.raceWith(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_CONE)),
      new GripperStateCommand(gripper, GripperState.OPEN),
      swerveControllerCommand6.raceWith(new WaitCommand(0.5).andThen(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP))),
      new GripperStateCommand(gripper, GripperState.CONE),
      new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP)


      
      );


    
  }
}
