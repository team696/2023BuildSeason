// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Gripper.GripperState;

public class GroundIntake extends CommandBase {
  
  /** Creates a new GroundIntake. */
  ArmSub armSub;
  Gripper gripper;

  public GroundIntake(ArmSub armSub, Gripper gripper) {
    this.gripper = gripper;
    this.armSub = armSub;
    addRequirements(armSub, gripper);

  }

  @Override
  public void initialize() {
    gripper.setClaw(GripperState.OPEN);
  }

  @Override
  public void execute() {
    gripper.moveGripper(0.8);
    armSub.armPresetPositions(ArmPositions.GROUND_PICKUP);

  }

  @Override
  public void end(boolean interrupted) {
    gripper.setClaw(GripperState.CONE);
    gripper.moveGripper(0);
    System.out.println("END");
  }

  @Override
  public boolean isFinished() {
    if(gripper.colorSensorDistance() > 200){
      return true;

    }
    else{
    return false;
    }
  }
}
