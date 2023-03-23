// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;

public class GroundIntake extends CommandBase {
  
  /** Creates a new GroundIntake. */
  ArmSub armSub;
  Gripper gripper;
  double timer;

  public GroundIntake(ArmSub armSub, Gripper gripper) {
    this.gripper = gripper;
    this.armSub = armSub;
    addRequirements(armSub, gripper);

  }

  @Override
  public void initialize() {
    timer = 0;
  }

  @Override
  public void execute() {
    timer++;
    armSub.armRotPresetPositions(ArmPositions.GROUND_PICKUP_ADAPTIVE);
    if (timer >= 20){
      gripper.moveGripper(1);

    }


  }

  @Override
  public void end(boolean interrupted) {
    gripper.moveGripper(0);
    System.out.println("END");
  }

  @Override
  public boolean isFinished() {
    if(gripper.getDistanceSensorDist() > 150){
      return true;

    }
    else{
    return false;
    }
  }
}
