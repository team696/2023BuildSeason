// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.util.Constants.ArmPositions;

public class AutoStow extends CommandBase {

  ArmSub armSub;
  Gripper gripper;
  ArmPositions armPosition = ArmPositions.STOWED_ADAPTIVE;
  boolean finished;

  /** Creates a new AutoStow. */
  public AutoStow(ArmSub armSub, Gripper gripper) {
    this.armSub = armSub;
    this.gripper = gripper;
    addRequirements(armSub, gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.jointRotPresetPositions(armPosition);
    armSub.armExtendPresetPositions(armPosition);
    if(armSub.getTelescopePos() <= ArmSub.MAX_EXTENSION*0.15){
      armSub.armRotPresetPositions(armPosition);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armSub.getArmEncoderPosition() - armSub.getArmRotGoal(armPosition)) <= 10;
  }
}
