// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.CANdleSub;

public class SetLedCommand extends CommandBase {
  CANdleSub ledsub;

  /** Creates a new SetLedCommand. */
  public SetLedCommand(CANdleSub ledsub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledsub);

    this.ledsub = ledsub;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledsub.setColor(GlobalVariables.gamePiece == 0);
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
