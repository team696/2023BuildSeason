// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSub extends SubsystemBase {
  
  private final CANdle m_candle = new CANdle(Constants.CANdle.id, "rio");
  private final int numLed = 48 + 32 + 32 + 48;
  private final int ledOffset = 8;
  /** Creates a new CANdle. */
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

  @Override
  public void periodic() {
    //m_candle.setLEDs(255, 0, 0);
   // m_candle.animate(new FireAnimation(1, 0.2, numLed, 0.6, 0.2, false, 8));
    //    m_candle.animate(new LarsonAnimation(0, 255, 46, 0, 1, 48, BounceMode.Front, 3, 120));
    // m_candle.animate(new LarsonAnimation(0, 255, 46, 0, 1, 32, BounceMode.Front, 3, 88));

    // m_candle.animate(new LarsonAnimation(0, 255, 46, 0, 1, 48, BounceMode.Front, 3, ledOffset));
    // m_candle.animate(new LarsonAnimation(0, 255, 46, 0, 1, 32, BounceMode.Front, 3, 56));
    // m_candle.animate(new SingleFadeAnimation(50, 2, 200, 0, 0.5, numLed));
    // m_candle.animate( new FireAnimation(0.5, 0.7, 48, 0.3, 0.1, true, ledOffset));
    // m_candle.animate( new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, numLed,ledOffset));
    m_candle.animate( new RainbowAnimation(1, 0.1, numLed, true, ledOffset));



    //m_candle.animate(new ColorFlowAnimation(255, 80, 160, 255, 0.6, numLed - 32, Direction.Forward, ledOffset));
   // m_candle.animate(new ColorFlowAnimation(255, 80, 160, 255, 0.6, 32, Direction.Backward, numLed + ledOffset));
   

  }
}

