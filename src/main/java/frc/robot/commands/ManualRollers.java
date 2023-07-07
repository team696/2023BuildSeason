// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.CANdleSub;
import frc.robot.subsystems.Gripper;

public class ManualRollers extends CommandBase {
  /** Creates a new AdaptiveOuttake. */
  Gripper gripper;
  CANdleSub candlesub;
  public ManualRollers(Gripper gripper, CANdleSub candlesub) {
    this.gripper = gripper;
    this.candlesub = candlesub;
    addRequirements(gripper, candlesub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(GlobalVariables.gamePiece == 0){
      gripper.moveGripper(-1);
    }
    else{
      gripper.moveGripper(1);
    }

    if(gripper.getGripperMotorCurrent() >= 100){
      candlesub.pickupLed();
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
