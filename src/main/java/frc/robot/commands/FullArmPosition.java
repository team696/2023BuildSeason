// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;

public class FullArmPosition extends CommandBase {
  ArmSub armSub;
  double armRotPos;
  double armExtendPos;
  boolean direction;



  /** Creates a new FullArmPosition. */
  public FullArmPosition(ArmSub armSub, double armRotPos, double armExtendPos, boolean direction) {
    this.armSub = armSub;
    this.armRotPos = armRotPos;
    this.armExtendPos = armExtendPos;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction){
    armSub.moveRotArmMotionMagic(armRotPos);
    if(armSub.getArmPosition() >= armRotPos*0.75){
      armSub.extendArmPosition(armExtendPos);
    }
    }
    else{
      armSub.extendArmPosition(armExtendPos);
      if(armSub.getTelescopePos() <= 30000){
      armSub.moveRotArmMotionMagic(armRotPos);
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
