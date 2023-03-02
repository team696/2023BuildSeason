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
import frc.robot.subsystems.Gripper.GripperState;

public class TestRed extends CommandBase {
  Swerve swerve;
  int timer;
  double xOffset1;
  double xOffset2;
  boolean reversed;
  SwerveControllerCommand swerveControllerCommand;

  /** Creates a new Test. */
  public TestRed(Swerve swerve, double xOffset1, double xOffset2, boolean reversed) {
    this.swerve = swerve;
    this.xOffset1 = xOffset1;
    this.xOffset2 = xOffset2;
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

    double[] pose = Constants.AutoConstants.RobotPositionsRed[swerve.tag][swerve.hor][swerve.height];

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
            new Pose2d(( pose[0])-xOffset1, pose[1], new Rotation2d(0)),
            new Pose2d(( pose[0])-xOffset2, pose[1], new Rotation2d(0))),
        config);
    System.out.println(pose[0]);
        // Trajectory exampleTrajectory =
        // TrajectoryGenerator.generateTrajectory(List.of(/* swerve.getAprilTagEstPosition(), */
        //     new Pose2d(2.8, 5.06, new Rotation2d(Math.PI)),
        //     new Pose2d(2.1, 5.06, new Rotation2d(Math.PI))),
        // config);


    var thetaController = new ProfiledPIDController(
        1.5, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        swerve::getAprilTagEstPosition,
        Constants.Swerve.swerveKinematics,
        new PIDController(2, 0.01, 0),
        new PIDController(2, 0.01, 0),
        thetaController,
        swerve::setModuleStates,
        swerve);

    swerve.m_fieldSim.getObject("traj2").setTrajectory(exampleTrajectory);
            // System.out.println("initialize");

    swerve.normalizeOdometry();
    swerveControllerCommand.schedule();
    System.out.println("TEST COMMAND INIT");


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("TEST COMMAND EXEC" + xOffset2);

        // System.out.println("execute ");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TEST COMMAND END");

    // System.out.println("end");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return swerveControllerCommand.isFinished();
      return swerveControllerCommand.isFinished();
  }
}
