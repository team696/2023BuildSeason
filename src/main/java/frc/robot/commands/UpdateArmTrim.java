// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;

public class UpdateArmTrim extends CommandBase {
  /** Creates a new UpdateArmTrim. */
  Joystick controller;
  int axis1;
  int axis2;

  public UpdateArmTrim(Joystick controller, int axis1) {
    this.controller = controller;
    this.axis1 = axis1;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GlobalVariables.armTrim = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GlobalVariables.armTrim += (controller.getRawAxis(axis1)/5);
    System.out.println(GlobalVariables.armTrim);
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