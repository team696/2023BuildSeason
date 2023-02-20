// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHolder extends ParallelCommandGroup {
  Swerve swerve;
  Gripper gripper;
  ArmSub armSub;
  /** Creates a new PlaceHolder. */
  public PlaceHolder(Swerve swerve, Gripper gripper, ArmSub armSub) {
    this.swerve = swerve;
    this.gripper = gripper;
    this.armSub = armSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Test(swerve), 
                new ArmPositionCommand(armSub, ArmPositions.MID_SCORE));
  }
}
