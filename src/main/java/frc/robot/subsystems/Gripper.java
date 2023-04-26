// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.Rev2mDistanceSensor;

import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  public TalonFX gripperFalcon;
  private SupplyCurrentLimitConfiguration limit;


  private Rev2mDistanceSensor distanceSensor;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperFalcon = new TalonFX(35, "Karen");
    gripperFalcon.configFactoryDefault();
    gripperFalcon.setNeutralMode(NeutralMode.Brake);


    limit = new SupplyCurrentLimitConfiguration(true, 50, 50, 0.5);
    // gripperFalcon.configSupplyCurrentLimit(limit);



    distanceSensor = new Rev2mDistanceSensor(Port.kMXP); 
    distanceSensor.setAutomaticMode(true);

  }

  public double getDistanceSensorDist(){
   return distanceSensor.getRange();
  }

  public double getGripperMotorCurrent(){
    return gripperFalcon.getSupplyCurrent();
  }

  public void moveGripper(double percent ){
    gripperFalcon.set(TalonFXControlMode.PercentOutput, percent);
  }

  @Override
  public void periodic() { 
    SmartDashboard.putNumber("Distance Sensor", getDistanceSensorDist());
    SmartDashboard.putNumber("GripperCurrent", gripperFalcon.getSupplyCurrent());
   
  }
}
