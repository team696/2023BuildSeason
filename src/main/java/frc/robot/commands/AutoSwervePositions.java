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

import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class AutoSwervePositions extends CommandBase {
  Swerve swerve;
  int timer;
  double x1;
  double x2;

  double rot1;

  double y1;
  double y2;

  double rot2;

  boolean reversed;
 private SwerveControllerCommand swerveControllerCommand;

  /** Creates a new Test. */
  public AutoSwervePositions(Swerve swerve, double x1, double y1, double rot1, double x2, double y2, double rot2, boolean reversed) {
    this.swerve = swerve;
    this.x1 = x1;
    this.x2 = x2;
    this.rot1 = rot1;
    this.rot2 = rot2;
    this.y1 = y1;
    this.y2 = y2;
    this.reversed = reversed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    TrajectoryConfig config = new TrajectoryConfig(
        0.5,
        0.5)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);


    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(x1, y1, new Rotation2d(rot1)),
            new Pose2d(x2, y2, new Rotation2d(rot2))),
        config);

    var thetaController = new ProfiledPIDController(
        0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        swerve::getAprilTagEstPosition,
        Constants.Swerve.swerveKinematics,
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController,
        swerve::setModuleStates,
        swerve);

    swerve.m_fieldSim.getObject("traj2").setTrajectory(exampleTrajectory);
            // System.out.println("initialize");

    swerve.normalizeOdometry();
    swerveControllerCommand.schedule();
    System.out.println("SWERVE COMMAND INIT");


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SWERVE COMMAND EXEC" );

        // System.out.println("execute ");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("SWERVE COMMAND END");

    // System.out.println("end");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return swerveControllerCommand.isFinished();
      return swerveControllerCommand.isFinished();
  }
}
