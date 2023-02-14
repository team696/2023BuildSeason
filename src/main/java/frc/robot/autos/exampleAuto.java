package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){


        Pose2d pose = s_Swerve.getPose();

        System.out.print(pose);
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);





        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(List.of(pose ,new Pose2d(0, 2,new Rotation2d(Math.PI)),  new Pose2d(1, 3,new Rotation2d( 0)), new Pose2d(2, 2, new Rotation2d(0))), config);
    

                                                        /* pose,
                                                        List.of(new Translation2d(6.5, 5),
                                                                 new Translation2d(6.54, 5.03)),
                                                       new Pose2d(
                                                                                           6.56,
                                                                                            5.076, 
                                                                                            Rotation2d.fromDegrees(0.0)),
                                                                                                config */


        var thetaController =
            new ProfiledPIDController(
                /* Constants.AutoConstants.kPThetaController */0.2, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);





        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


                

            
        addCommands(
            // new InstantCommand(() -> s_Swerve.normalizeOdometry()),
            swerveControllerCommand
        );
    }
}