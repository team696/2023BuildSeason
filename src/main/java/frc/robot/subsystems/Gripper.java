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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

public class Gripper extends SubsystemBase {

  private TalonFX gripperFalcon;
  //private Rev2mDistanceSensor distanceSensor;

  /** Creates a new Gripper. */
  public Gripper() {
    gripperFalcon = new TalonFX(35, "Karen");
    gripperFalcon.configFactoryDefault();
    gripperFalcon.setNeutralMode(NeutralMode.Brake);

    //if (RobotBase.isReal()){
    //  distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy); 
    //  distanceSensor.setAutomaticMode(true);
    //}

    this.setDefaultCommand(this.slowIntake());

    if (Constants.useShuffleboard)
      SmartDashboard.putData(this);
  }

  
  public double getGripperMotorCurrent(){
    return gripperFalcon.getSupplyCurrent();
  }

  public void moveGripper(double percent ){
    gripperFalcon.set(TalonFXControlMode.PercentOutput, percent);
  }

  public double getDistanceSensorM(){
    return -1;//RobotBase.isReal() ? distanceSensor.getRange() / 1000 : -1;
 }

 public CommandBase slowIntake() {
  return this.runEnd(() -> {if (ArmSub.gamePiece == 0) moveGripper(-0.075); else moveGripper(0.07);}, ()->moveGripper(0));
 }

 public CommandBase spit() {
  return this.runEnd(()->{if (ArmSub.gamePiece == 0) moveGripper(0.65); else moveGripper(-0.6);}, ()->moveGripper(0));
 }

 public CommandBase intake() {
  return this.runEnd(() -> {if (ArmSub.gamePiece == 0) moveGripper(-1); else moveGripper(1);}, ()->moveGripper(0));
 }

 public void cancel() {
    this.getCurrentCommand().cancel();
 }

  @Override
  public void periodic() {    
    if (gripperFalcon.getSupplyCurrent() > 30) CANdleSub.override = true;
    //if (!RobotBase.isReal()) return;
    //if (distanceSensor.getRange() <= 0) { // Reinitialize on loss of connection?
    //  distanceSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighAccuracy); 
    //  distanceSensor.setAutomaticMode(true);
    //  distanceSensor.setEnabled(true);
    //}
  }

  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Gripper Current", ()->gripperFalcon.getSupplyCurrent(), null);
    builder.addDoubleProperty("Distance (m)", () -> {return getDistanceSensorM();}, null);
  }
}
