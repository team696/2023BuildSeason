// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  DoubleSolenoid cubeSolenoid;
  DoubleSolenoid coneSolenoid;
  public PneumaticsControlModule module;
  Compressor compressor;

  public enum GripperState{
    OPEN, CONE, CUBE
  }
  public GripperState gripperState = GripperState.OPEN;
  /** Creates a new Gripper. */
  public Gripper() {
    gripperMotor = new CANSparkMax(42, MotorType.kBrushless);
    gripperMotor.restoreFactoryDefaults();
    gripperMotor.setIdleMode(IdleMode.kBrake);

    compressor = new Compressor(PneumaticsModuleType.REVPH);
    compressor.enableDigital();

    cubeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 2);
    coneSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 3);

    module = new PneumaticsControlModule(1);
    module.clearAllStickyFaults();
    

  }

  public void moveGripper(double percent ){
    gripperMotor.set(percent);
  }

  public void setClaw(GripperState state){
    switch(state){
      case OPEN:
      cubeSolenoid.set(Value.kForward);
      coneSolenoid.set(Value.kForward);
      break;

      case CONE:
      cubeSolenoid.set(Value.kReverse);
      coneSolenoid.set(Value.kReverse);
      break;

      case CUBE:
      // cubeSolenoid.set(Value.kForward);
      // coneSolenoid.set(Value.kOff);
      break;
  }
}


  @Override
  public void periodic() { }
}
