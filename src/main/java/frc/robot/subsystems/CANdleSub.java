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
//   public enum AnimationTypes {
//     ColorFlow,
//     Fire,
//     Larson,
//     Rainbow,
//     RgbFade,
//     SingleFade,
//     Strobe,
//     Twinkle,
//     TwinkleOff,
//     SetAll,
//     Empty
// }

// public Animation colorflow = new ColorFlowAnimation(ledOffset, numLed, ledOffset)
// private AnimationTypes m_currentAnimation;


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

  public void disabledLed(){
    m_candle.animate(new SingleFadeAnimation(0, 255, 0, 0, 0.7, numLed, ledOffset));
    // m_candle.animate(new ColorFlowAnimation(128, 20, 70, 0, 0.7, numLed, Direction.Forward, 8));

  }

  public void enabledLed(){
    m_candle.animate(new SingleFadeAnimation(230, 10, 10, 0, 0.7, numLed, ledOffset));

  }



  @Override
  public void periodic() {
    

  }
}

