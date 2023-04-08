// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;

public class UprightConeIntake extends CommandBase {
  /** Creates a new GroundIntake. */
  ArmSub armSub;
  Gripper gripper;
  public UprightConeIntake(ArmSub armSub, Gripper gripper) {
    this.armSub = armSub;
    this.gripper = gripper;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      armSub.moveGripperJointPosition(46000);
      armSub.moveTelescopeArmPosition(0);
      armSub.moveRotArmPosition(44);
      
      if((gripper.getDistanceSensorDist() <= 12)){
      gripper.moveGripper(0);

      }
      else{
        gripper.moveGripper(-1);

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
