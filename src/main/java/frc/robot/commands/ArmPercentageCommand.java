// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;

public class ArmPercentageCommand extends CommandBase {
  ArmSub armSub; 
  int axis1;
  int axis2;
  Joystick controller;
  
  /** Creates a new ArmPercentageCommand. */
  public ArmPercentageCommand(ArmSub armSub, int axis1, int axis2, Joystick controller) {
    this.armSub = armSub;
    this.axis1 = axis1;
    this.axis2 = axis2;
    this.controller = controller;
    addRequirements(armSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.moveRotArmPercentOutput(controller.getRawAxis(axis1) - controller.getRawAxis(axis2) );
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
