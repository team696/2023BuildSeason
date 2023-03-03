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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Test;
import frc.robot.commands.TurnInPlace;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Gripper.GripperState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class LeftSideAutoRed extends SequentialCommandGroup {
  Swerve swerve;
  ArmSub armSub;
  Gripper gripper;
  SwerveControllerCommand swerveControllerCommand1;
  SwerveControllerCommand swerveControllerCommand2;
  SwerveControllerCommand swerveControllerCommand3;
  SwerveControllerCommand swerveControllerCommand4;
  SwerveControllerCommand swerveControllerCommand5;
  SwerveControllerCommand swerveControllerCommand6;
  Joystick controller;

MinMax minMax;
    TrajectoryConstraint constraints;

  /** Creates a new AutoPlaceTest. */
  public LeftSideAutoRed(Swerve swerve, ArmSub armSub, Gripper gripper) {
    this.swerve = swerve;
    this.armSub = armSub;
    this.gripper = gripper;
    controller = new Joystick(1);
    
           
    // constraints.getMinMaxAccelerationMetersPerSecondSq(null, 0, 0)
    TrajectoryConfig config = new TrajectoryConfig(
        1.7,
        1.7)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
        config.addConstraint(new CentripetalAccelerationConstraint(0.3));

        TrajectoryConfig placeConfig = new TrajectoryConfig(
            0.95,
            1.3)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
            config.addConstraint(new CentripetalAccelerationConstraint(0.3));
    

    TrajectoryConfig configrev = new TrajectoryConfig(
        3,
        3)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
        configrev.addConstraint(new CentripetalAccelerationConstraint(0.3));

        TrajectoryConfig configrevdfast = new TrajectoryConfig(
        2,
        2)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
        configrev.addConstraint(new CentripetalAccelerationConstraint(0.3));

        Trajectory traj1 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(13.54, 5.0, new Rotation2d(0)),
            new Pose2d(14.24, 5.0, new Rotation2d(0))),
        placeConfig);

        Trajectory traj2 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(14.34, 5.15, new Rotation2d(0)),
            new Pose2d(13.74, 5.15, new Rotation2d(0))),
        configrev);

        Trajectory traj3 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(13.74, 5.15, new Rotation2d(0)),
            new Pose2d(12.54, 4.75, new Rotation2d(0))),
        configrevdfast);

        Trajectory traj4 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(12.54, 5.15, new Rotation2d(0)),
            new Pose2d(13.74, 5.15, new Rotation2d(0))),
        config);

        Trajectory traj5 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(13.54, 5.15, new Rotation2d(0)),
            new Pose2d(14.69, 5.0, new Rotation2d(0))),
        config);
        Trajectory traj6 =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(14.44, 5.15, new Rotation2d(0)),
            new Pose2d(13.74, 4.8, new Rotation2d(0))),
        configrev);

      
          


    var thetaController = new ProfiledPIDController(
        1, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand1 = new SwerveControllerCommand(
        traj1,
        swerve::getAprilTagEstPosition,
        Constants.Swerve.swerveKinematics,
        new PIDController(2, 0, 0),
        new PIDController(2, 0, 0),
        thetaController,
        swerve::setModuleStates,
        swerve);

        swerveControllerCommand2 = new SwerveControllerCommand(
          traj2,
          swerve::getAprilTagEstPosition,
          Constants.Swerve.swerveKinematics,
          new PIDController(2, 0, 0),
          new PIDController(2, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve);

          swerveControllerCommand3 = new SwerveControllerCommand(
            traj3,
            swerve::getAprilTagEstPosition,
            Constants.Swerve.swerveKinematics,
            new PIDController(1.7, 0, 0),
            new PIDController(1.7, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
            swerveControllerCommand4 = new SwerveControllerCommand(
              traj4,
              swerve::getPose,
              Constants.Swerve.swerveKinematics,
              new PIDController(1, 0, 0),
              new PIDController(1, 0, 0),
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
      new GripperStateCommand(gripper, GripperState.OPEN).raceWith(new TeleopSwerve(swerve, controller, 0, 0, 0, false, false)),
      new InstantCommand(() -> swerve.normalizeOdometry()),
      swerveControllerCommand3.raceWith(/* new HoldArmPos(armSub, ArmPositions.STOWED) */new WaitCommand(0.5).andThen(new HoldArmPos(armSub, ArmPositions.STOWED).alongWith(new GripperStateCommand(gripper, GripperState.CONE)))),
      new TurnInPlace(swerve, true , false, 178),
      new AutoLockToGamePiece(swerve, 0, 0, false, false , 0).raceWith(new GroundIntake(armSub, gripper)),
      new WaitCommand(0.4 ).raceWith(new TeleopSwerve(swerve, controller, 0, 0, 0, false, false)),
      new TurnInPlace(swerve, true , false, 0),
      swerveControllerCommand4.raceWith(new HoldArmPos(armSub, ArmPositions.STOWED)),
       new WaitCommand(0.5).raceWith(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_CONE)),
      swerveControllerCommand5.raceWith(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_CONE)),
      new GripperStateCommand(gripper, GripperState.OPEN).raceWith(new TeleopSwerve(swerve, controller, 0, 0, 0, false, false)),
      swerveControllerCommand6.raceWith(new WaitCommand(1).andThen(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP))),
      new GripperStateCommand(gripper, GripperState.CONE),
      new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP)


      
      );


    
  }
}
