// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.util.Constants.ArmPositions;

public class AutoPlace extends CommandBase {
  /** Creates a new AdaptiveArmMovement. */
  ArmSub armSub;
  Gripper gripper;
  ArmPositions armPosition;
  boolean finished;
  int timer;
  public AutoPlace(ArmSub armSub, Gripper gripper, ArmPositions armPosition) {
    this.armSub = armSub;
    this.armPosition = armPosition;
    this.gripper = gripper;
    addRequirements(armSub, gripper);
    finished = false;
    timer = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    timer = 0;
    armSub.resetPID();
    if (ArmSub.gamePiece == 0) {
      gripper.moveGripper(-0.5);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.armRotPresetPositions(armPosition);
    armSub.jointRotPresetPositions(armPosition);
    if(Math.abs(armSub.getArmEncoderPosition() - armSub.getArmRotGoal(armPosition)) < 20){
      armSub.armExtendPresetPositions(armPosition);
      if (Math.abs(armSub.getTelescopePos() - armSub.getArmExtendGoal(armPosition)) <= 1000 && Math.abs(armSub.getArmEncoderPosition() - armSub.getArmRotGoal(armPosition)) < 8) {
      timer++;
      if(timer>=10) {
        if(ArmSub.gamePiece == 0){
          gripper.moveGripper(0.7);
        } else {
          gripper.moveGripper(-0.6);
        }
        if (timer >= 20){
          finished = true;
        }
      }
    }
  }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripper.moveGripper(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
