// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotor = new CANSparkMax(42, MotorType.kBrushless);
    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kBrake);
  }

  public void moveGripper(double percent ){
    gripperMotor.set(percent);;
  }


  @Override
  public void periodic() { }
}
