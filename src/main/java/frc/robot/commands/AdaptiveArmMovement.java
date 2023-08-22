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
  public AdaptiveArmMovement(ArmSub armSub, ArmPositions armPosition) {
    this.armSub = armSub;
    this.armPosition = armPosition;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // TODO:: MAKE SURE THIS WORKS!! -> ADAPT OTHER POSITIONS INTO THIS -> STUPID THEY ARE IN OTHER FILES
   // if(armPosition == ArmPositions.STOWED_ADAPTIVE){
     // armSub.armExtendPresetPositions(armPosition);
     // armSub.jointRotPresetPositions(armPosition);
     // if(armSub.getTelescopePos() <= 10000 && armSub.getGripperJointPos() <= 5000){
     //   armSub.armRotPresetPositions(armPosition);
     // }
    //} else {
      if (armSub.armRotGoal > armSub.getArmEncoderPosition()) { // GOING UP
        armSub.armRotPresetPositions(armPosition);
        if (armSub.getArmEncoderPosition() / armSub.armRotGoal > 0.7){ 
        // if(armSub.getArmEncoderPosition() >= armSub.armRotGoal*0.15)
          armSub.jointRotPresetPositions(armPosition);
          armSub.armExtendPresetPositions(armPosition);
        } 
      } else {
        armSub.jointRotPresetPositions(armPosition);
        armSub.armExtendPresetPositions(armPosition);
        if (Math.abs(armSub.getTelescopePos() - armSub.armExtendGoal) < ArmSub.MAX_EXTENSION * 0.3){ 
          armSub.armRotPresetPositions(armPosition);
        } 
      }
   // }
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
