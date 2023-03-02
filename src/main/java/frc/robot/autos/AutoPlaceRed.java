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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.GripperStateCommand;
import frc.robot.commands.HoldArmPos;
import frc.robot.commands.HoldArmPosAuto;
import frc.robot.commands.Test;
import frc.robot.commands.TestRed;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Gripper.GripperState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceRed extends SequentialCommandGroup {
  Swerve swerve;
  ArmSub armSub;
  Gripper gripper;
  SwerveControllerCommand swerveControllerCommand;

  /** Creates a new AutoPlaceTest. */
  public AutoPlaceRed(Swerve swerve, ArmSub armSub, Gripper gripper) {
    this.swerve = swerve;
    this.armSub = armSub;
    this.gripper = gripper;
    
    addCommands(
      // new InstantCommand(() -> swerve.normalizeOdometry()),
      // new Test(swerve, 0.7, 0).raceWith(new HoldArmPos(armSub, ArmPositions.MID_SCORE)),
      // new GripperStateCommand(gripper, GripperState.CONE),
      // new HoldArmPos(armSub, ArmPositions.MID_SCORE).raceWith  (new WaitCommand(1)),
      // new Test(swerve, 0, 0.7).raceWith(new HoldArmPos(armSub, ArmPositions.MID_SCORE)),
      // new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).raceWith(new WaitCommand(0.1)),
      // new GripperStateCommand(gripper, GripperState.OPEN)
      new InstantCommand(() -> swerve.normalizeOdometry()),
      new WaitCommand(0.5).raceWith(new HoldArmPosAuto(armSub)),
      new TestRed(swerve, 0.6, 0, false).raceWith(new HoldArmPosAuto(armSub)),
      new GripperStateCommand(gripper, GripperState.OPEN).raceWith(new HoldArmPosAuto(armSub)),
      new TestRed(swerve, 0.2, 0.7, true).raceWith(new WaitCommand(1.5).andThen(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP))),
      new GripperStateCommand(gripper, GripperState.CONE).raceWith(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP)),
      new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).raceWith(new WaitCommand(1))

      
      );
    
// 16.54
    
  }
}
