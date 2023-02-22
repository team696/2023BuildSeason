// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Gripper.GripperState;

public class GripperStateCommand extends CommandBase {
  Gripper gripper;
  GripperState state;
  int timer;
  /** Creates a new GripperStateCommand. */
  public GripperStateCommand(Gripper gripper, GripperState state) {
    this.gripper = gripper;
    this.state = state;
    addRequirements(gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    gripper.setClaw(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer ++;
    System.out.println("GRIPPING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DONE GRIPPING");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 2;
  }
}
