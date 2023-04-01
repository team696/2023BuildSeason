// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSub;

public class ArmExtendTest extends CommandBase {
  ArmSub armsub;
  Joystick controller;
  /** Creates a new ArmExtendTest. */
  public ArmExtendTest(ArmSub armsub, Joystick controller) {
    this.armsub = armsub;
    this.controller = controller;
    addRequirements(armsub);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRawButton(17)){
    armsub.moveRotArmPercentOutput(controller.getRawAxis(0));
    }
    else{
      armsub.moveTelescopeArmPercentOutput(controller.getRawAxis(0));
    }
    // armsub.extendArmPosition(1000);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armsub.moveRotArmPercentOutput(0);
    armsub.moveTelescopeArmPercentOutput(0);
    // armsub.extendArmPosition(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
