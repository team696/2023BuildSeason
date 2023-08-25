// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.util.Constants.ArmPositions;

public class AdaptiveArmMovement extends CommandBase {
  /** Creates a new AdaptiveArmMovement. */
  ArmSub armSub;
  ArmPositions armPosition;
  int gamepiece_override = -1;
  public AdaptiveArmMovement(ArmSub armSub, ArmPositions armPosition) {
    this.armSub = armSub;
    this.armPosition = armPosition;
    gamepiece_override = -1;
    addRequirements(armSub);
  }

  public AdaptiveArmMovement(ArmSub armSub, ArmPositions armPosition, int gamepiece_o) {
    this.armSub = armSub;
    this.armPosition = armPosition;
    gamepiece_override = gamepiece_o;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
      if (gamepiece_override != -1)
        ArmSub.gamePiece = gamepiece_override;
      if (armSub.getArmRotGoal(armPosition) > armSub.getArmEncoderPosition()) { // GOING UP
        armSub.armRotPresetPositions(armPosition);
        if (armSub.getArmEncoderPosition() / armSub.getArmRotGoal(armPosition) > 0.7 || Math.abs(armSub.getArmEncoderPosition() - armSub.getArmRotGoal(armPosition)) < 7){ 
          armSub.jointRotPresetPositions(armPosition);
          armSub.armExtendPresetPositions(armPosition);
        } 
      } else {
        armSub.jointRotPresetPositions(armPosition);
        armSub.armExtendPresetPositions(armPosition);
        if (Math.abs(armSub.getTelescopePos() - armSub.getArmExtendGoal(armPosition)) < ArmSub.MAX_EXTENSION * 0.05){ 
          armSub.armRotPresetPositions(armPosition);
        } 
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
