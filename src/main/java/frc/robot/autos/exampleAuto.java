package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve, int tag, int horizontal, int height){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    0.5,
                    0.25)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);

        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(List.of(
        //             new Pose2d(2.55, 2.73,new Rotation2d(Math.PI)), 
        //             new Pose2d(2.4, 2.73,new Rotation2d(Math.PI))),
        //         config);

        double[] pose = Constants.AutoConstants.RobotPositions[tag][horizontal][height];


        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(List.of(
                    new Pose2d(11, 3.57, new Rotation2d(0)), 
                    new Pose2d(12.47, 3.57, new Rotation2d(0))),
                config);

        var thetaController =
            new ProfiledPIDController(
               01, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getAprilTagEstPosition,
                Constants.Swerve.swerveKinematics,
                new PIDController(2,2, 0),
                new PIDController(2, 2,  0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        s_Swerve.m_fieldSim.getObject("traj").setTrajectory(exampleTrajectory);
                
        addCommands(
            new InstantCommand(() -> s_Swerve.normalizeOdometry()),
            swerveControllerCommand);
    }
}