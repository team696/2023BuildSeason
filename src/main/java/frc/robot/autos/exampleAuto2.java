package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto2 extends SequentialCommandGroup {
            


    public exampleAuto2(Swerve s_Swerve){
       
        TrajectoryConfig config =
            new TrajectoryConfig(
                    0.5,
                    0.5)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
        
        double[] pose = Constants.AutoConstants.RobotPositions[s_Swerve.tag][s_Swerve.height][s_Swerve.hor];

        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(List.of(
                new Pose2d(pose[0]+0.7, pose[1], new Rotation2d(Math.PI)),
                new Pose2d(pose[0], pose[1], new Rotation2d(Math.PI))),
            config);
        
        var thetaController =
            new ProfiledPIDController(
                0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        s_Swerve.m_fieldSim.getObject("traj2").setTrajectory(exampleTrajectory);
             
        addCommands(
            new InstantCommand(() -> s_Swerve.normalizeOdometry()),
            swerveControllerCommand);
    }

    
    
}