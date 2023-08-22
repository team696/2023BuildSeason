// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;

public class GroundIntake extends CommandBase {
  /** Creates a new GroundIntake. */
  ArmSub armSub;
  Gripper gripper;
  public GroundIntake(ArmSub armSub, Gripper gripper) {
    this.armSub = armSub;
    this.gripper = gripper;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override //TODO: MAKE ME IN ADAPTIVE
  public void execute() {
    if(ArmSub.gamePiece == 0){
      armSub.moveGripperJointPosition(17422);
      armSub.moveTelescopeArmPosition(8000);
      armSub.moveRotArmPosition(0);
      gripper.moveGripper(-1);
    }else{
      armSub.moveGripperJointPosition(13288);
      armSub.moveTelescopeArmPosition(8000);
      armSub.moveRotArmPosition(0);
      gripper.moveGripper(0.8);
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
    return false;
  }
}
