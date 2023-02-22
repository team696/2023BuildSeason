// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceTest extends SequentialCommandGroup {
  Swerve swerve;
  ArmSub armSub;
  SwerveControllerCommand swerveControllerCommand;

  /** Creates a new AutoPlaceTest. */
  public AutoPlaceTest(Swerve swerve, ArmSub armSub) {
    this.swerve = swerve;
    this.armSub = armSub;
    TrajectoryConfig config = new TrajectoryConfig(
      0.5,
      0.5)
      .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);

  double[] pose = Constants.AutoConstants.RobotPositions[swerve.tag][swerve.hor][swerve.height];

  Trajectory exampleTrajectory =
      TrajectoryGenerator.generateTrajectory(List.of(
          new Pose2d(pose[0]+0.7, pose[1], new Rotation2d(Math.PI)),
          new Pose2d(pose[0], pose[1], new Rotation2d(Math.PI))),
      config);

  var thetaController = new ProfiledPIDController(
      0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  swerveControllerCommand = new SwerveControllerCommand(
      exampleTrajectory,
      swerve::getPose,
      Constants.Swerve.swerveKinematics,
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      thetaController,
      swerve::setModuleStates,
      swerve);

  swerve.m_fieldSim.getObject("traj2").setTrajectory(exampleTrajectory);
          System.out.println("initialize");

    
    addCommands(
      new InstantCommand(() -> swerve.normalizeOdometry()),
      swerveControllerCommand.alongWith(new ArmPositionCommand(armSub, ArmPositions.MID_SCORE))


    );
  }
}
