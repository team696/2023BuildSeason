// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class CANdleSub extends SubsystemBase {
  
  private final CANdle m_candle = new CANdle(Constants.CANdle.id, "rio");
  private final int numLed = 200;
  private final int ledOffset = 0;
  public static boolean override = false;
  public CANdleSub() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.On;
    configAll.enableOptimizations = true;
    configAll.v5Enabled = true;
    m_candle.configAllSettings(configAll, 100);
  }

  public void disabledLed(){
    m_candle.animate(new SingleFadeAnimation(200, 0, 0, 0, 0.6, numLed, ledOffset), 0);
  }

  public void enabledLed(){
    m_candle.animate(new SingleFadeAnimation(230, 10, 10, 0, 0.7, numLed, ledOffset));
  }

  @Override
  public void periodic() {
    if (!RobotBase.isReal()) return;

    if (DriverStation.isTeleopEnabled()) {
      m_candle.clearAnimation(0);
      if (ArmSub.gamePiece==0) 
        m_candle.setLEDs(255, 45, 0, 0, ledOffset, numLed);
      else  
        m_candle.setLEDs(111, 3, 252, 0, ledOffset, numLed);
      if (override)
        m_candle.setLEDs(0,255,0,0,ledOffset, numLed);
      
    } else if (DriverStation.isDisabled()) {
      disabledLed();
    }  else if (DriverStation.isAutonomousEnabled()) {
      enabledLed();
    }
    override = false;
  }
}

