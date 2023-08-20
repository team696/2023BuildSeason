// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.util.Constants.ArmPositions;

public class AutoShootCone extends CommandBase {
  /** Creates a new AdaptiveArmMovement. */
  ArmSub armSub;
  Gripper gripper;
  ArmPositions armPosition;
  boolean finished;
  int timer;
  public AutoShootCone(ArmSub armSub, Gripper gripper) {
    this.armSub = armSub;
    this.gripper = gripper;
    addRequirements(armSub, gripper);
    finished = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    timer = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
      armSub.moveRotArmPosition(90);
      armSub.moveGripperJointPosition(11000);

      if(armSub.getArmEncoderPosition() >= 80){
       
          timer++;
          if(/* gripper.getDistanceSensorDist() <= 12 */ timer>=15){
            if(ArmSub.gamePiece == 0){
              gripper.moveGripper(1);
            }
            else{
              gripper.moveGripper(-1);
  
            }
            if (timer >= 30){
              gripper.moveGripper(0);
              finished = true;


            }
          }
          else{
            gripper.moveGripper(0);
          }
        }
        else{
          finished = false;
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
