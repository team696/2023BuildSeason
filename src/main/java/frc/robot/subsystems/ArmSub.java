// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSub extends SubsystemBase {
  public TalonFX leftArm;
  public TalonFX rightArm; 
  /** Creates a new ArmSub. */
  public ArmSub() {

    leftArm = new TalonFX(30);
    rightArm = new TalonFX(31);

    leftArm.configFactoryDefault();
    leftArm.setNeutralMode(NeutralMode.Brake);


    rightArm.configFactoryDefault();
    rightArm.setInverted(InvertType.InvertMotorOutput);
    rightArm.follow(leftArm);
    rightArm.setNeutralMode(NeutralMode.Brake);


  }
/**
 * Moves the arm by percent output.
 * @param percent 0.0 - 1.0.
  */
  public void moveArmPercentOutput(double percent){
    leftArm.set(TalonFXControlMode.PercentOutput, percent);
  }

  public void moveArmPosition(double position){
    leftArm.set(ControlMode.Position, position);
  }

  public double getArmPosition(){
    return leftArm.getSelectedSensorPosition();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
