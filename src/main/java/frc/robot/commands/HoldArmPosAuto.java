// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ArmSub;

public class HoldArmPosAuto extends CommandBase {
  ArmSub armSub;
  // GlobalVariables.ArmPositions armPos;
  int timer;

  /** Creates a new ArmPositionCommand. */
  public HoldArmPosAuto(ArmSub armSub) {
 this.armSub = armSub;
  
 addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    System.out.println("ARM COMMAND INIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer ++;
    double armpos = Constants.ArmPositions[GlobalVariables.gamePiece][GlobalVariables.height];
    armSub.moveRotArmPosition(armpos);
    System.out.println("ARM COMMAND EXEC");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ARM COMMAND END");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
