// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {

  CANSparkMax gripperMotor;
  DoubleSolenoid cubeSolenoid;
  Solenoid coneSolenoid;
  public PneumaticsControlModule module;
  Compressor compressor;
  public final  I2C.Port i2cPort = I2C.Port.kMXP;

  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
;
  public Color detectedColor;

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
    // coneSolenoid = new DoubleS/olenoid(PneumaticsModuleType.REVPH, 1, 3);
    coneSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

    module = new PneumaticsControlModule(1);
    module.clearAllStickyFaults();
    
    detectedColor = colorSensor.getColor();

  }

  public void getColorSensor(){
    detectedColor = colorSensor.getColor();
 }
  public double getRed(){
    getColorSensor();
    return detectedColor.red;
  }

  public double getBlue(){
    getColorSensor();
    return detectedColor.blue;
  }

  public double getGreen(){
    getColorSensor();
    return detectedColor.green;
  }

  public double colorSensorDistance(){
    return colorSensor.getProximity();
  }


  public void moveGripper(double percent ){
    gripperMotor.set(percent);
  }

  public void setClaw(GripperState state){
    switch(state){
      case OPEN:
      cubeSolenoid.set(Value.kReverse);
      coneSolenoid.set(false);
      break;

      case CONE:
      cubeSolenoid.set(Value.kForward);
      coneSolenoid.set(false);
      break;

      case CUBE:
      cubeSolenoid.set(Value.kOff);
      coneSolenoid.set(true);
      break;
  }
}


  @Override
  public void periodic() { 
    SmartDashboard.putNumber("RED", colorSensor.getRed());
    SmartDashboard.putNumber("BLUE", colorSensor.getBlue());
    SmartDashboard.putNumber("GREEN", colorSensor.getGreen());
    SmartDashboard.putNumber("Distance", colorSensor.getProximity());
    // System.out.print(colorSensor.getBlue());
  }
}
