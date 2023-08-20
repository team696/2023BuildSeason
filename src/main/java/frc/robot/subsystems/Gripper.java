// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class Gripper extends SubsystemBase {

  private TalonFX gripperFalcon;
  private Rev2mDistanceSensor distanceSensor;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperFalcon = new TalonFX(35, "Karen");
    gripperFalcon.configFactoryDefault();
    gripperFalcon.setNeutralMode(NeutralMode.Brake);

    if (RobotBase.isReal()){
      distanceSensor = new Rev2mDistanceSensor(Port.kOnboard); 
      distanceSensor.setAutomaticMode(true);
      distanceSensor.setDistanceUnits(Unit.kMillimeters);
    }

    SmartDashboard.putData(this);
  }

  public double getGripperMotorCurrent(){
    return gripperFalcon.getSupplyCurrent();
  }

  public void moveGripper(double percent ){
    gripperFalcon.set(TalonFXControlMode.PercentOutput, percent);
  }

  public double getDistanceSensorM(){
    return RobotBase.isReal() ? distanceSensor.getRange() / 1000 : 0;
 }

  @Override
  public void periodic() {    
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Gripper Current", ()->gripperFalcon.getSupplyCurrent(), null);
    builder.addDoubleProperty("Distance (m)", () -> {return getDistanceSensorM();}, null);
  }
}
