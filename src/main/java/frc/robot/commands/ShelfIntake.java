// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.util.Constants.ArmPositions;
public class ShelfIntake extends CommandBase {
  /** Creates a new AdaptiveArmMovement. */
  ArmSub armSub;
  Gripper gripper;
  public ShelfIntake(ArmSub armSub, Gripper gripp) {
    this.armSub = armSub;
    this.gripper = gripp;
    addRequirements(armSub, gripp);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gripper.moveGripper(-1);

      armSub.armRotPresetPositions(ArmPositions.SHELF_PICKUP_ADAPTIVE);
      armSub.jointRotPresetPositions(ArmPositions.SHELF_PICKUP_ADAPTIVE);

      if(armSub.getArmEncoderPosition() >= armSub.armRotGoal*0.75){
        armSub.armExtendPresetPositions(ArmPositions.SHELF_PICKUP_ADAPTIVE);
      }
    }

  @Override
  public void end(boolean interrupted) {
    // gripper.moveGripper(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
