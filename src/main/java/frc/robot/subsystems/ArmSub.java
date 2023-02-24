// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ArmSub extends SubsystemBase {
  public TalonFX leftArm;
  public TalonFX rightArm; 
  public PIDController pidLoop;
  public TalonSRX encorderTalon;
  public CANCoder testCanCoder;
  public TalonSRXFeedbackDevice encoder; 
  public double maxSpeedFor;
  public double maxSpeedRev;
  


  /** Creates a new ArmSub. */
  public ArmSub() {

  
    testCanCoder = new CANCoder(14, "Abu");
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    maxSpeedFor = 0.3;
    maxSpeedRev = 0.6;


    // pidLoop = new PIDController(0.006, 0.003, 0.001);  
    pidLoop = new PIDController(0.012, 0.003, 0.00);
    // pidLoop.

    leftArm = new WPI_TalonFX(20, "Abu");
    rightArm = new WPI_TalonFX(21, "Abu");

    leftArm.configFactoryDefault();
      leftArm.setNeutralMode(NeutralMode.Brake);
      leftArm.configPeakOutputForward(maxSpeedFor);
      leftArm.configPeakOutputReverse(-maxSpeedRev);
      leftArm.config_kP(0, 0.05);
      leftArm.config_kI(0, 0.0);
      leftArm.config_kD(0, 0.0);
      leftArm.config_kF(0, 0.06);
      leftArm.setSensorPhase(true);
      leftArm.setInverted(InvertType.None);
      leftArm.configRemoteFeedbackFilter(testCanCoder, 0);
      leftArm.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
 


    rightArm.configFactoryDefault();
      rightArm.configPeakOutputForward(maxSpeedFor);
      rightArm.configPeakOutputReverse(-maxSpeedRev);
      rightArm.setSensorPhase(true);
      rightArm.setInverted(InvertType.FollowMaster);
      rightArm.config_kP(0, 0.05);
      rightArm.config_kI(0, 0.0);
      rightArm.config_kD(0, 0.0);
      rightArm.config_kF(0, 0.06);
      rightArm.setNeutralMode(NeutralMode.Brake);
      rightArm.follow(leftArm);
    

  }
 

 
/**
 * Moves the arm by percent output.
 * @param percent 0.0 - 1.0.
  */
  public void moveArmPercentOutput(double percent){
    leftArm.set(TalonFXControlMode.PercentOutput, percent);
  }

   /** 
    * Sets the encoders within the arm Falcons to the current CANCoder readout.
    * 
    */
  private void resetToAbsolute(){
    double absolutePosition = Conversions.degreesToFalcon(testCanCoder.getAbsolutePosition(), (56));
    leftArm.setSelectedSensorPosition(absolutePosition);
}
/**
 * Moves the arm to a dedicated degree.
 * @param position Desired arm position in degrees
 */
  public void moveArmPosition(double position){
    leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), position));
  }
/**
 * Moves the arm to a dedicated preset positions
 * @param position The dedicated position enum, current options are GROUND_PICKUP, STOWED, GROUND_SCORE, MID_SCORE, HIGH_SCORE, and SHELF_PICKUP
  */
  public void armPresetPositions(GlobalVariables.ArmPositions position){
    switch(position){

      case GROUND_PICKUP:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.grndIntakePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.grndIntakePosValue));
      break;

      case STOWED:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.stowedPosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.stowedPosValue));

      break;

      case GROUND_SCORE: 
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.grndScorePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.grndScorePosValue));

      break;

      case MID_SCORE:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.midScorePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.midScorePosValue));

      break;

      case HIGH_SCORE:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.highScorePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.highScorePosValue));

      break;

      case SHELF_PICKUP:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.shelfIntakePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.shelfIntakePosValue));

      break;

      default:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(getArmPosition(), Constants.shelfIntakePosValue));
      // System.out.println("CancoderPos" + getArmPosition() + "PID Loop Output"+pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.shelfIntakePosValue));

      break;
    }
  }
/**
 * Returns current arm position
 * @return CANcoder arm position in degrees.
  */
  public double getArmPosition(){
    return testCanCoder.getAbsolutePosition();
  }
/**
 * Returns current arm motor position.
 * @return Current arm falcon position unfiltered.
  */
  public double getbackArmMotorPosition(){
    return leftArm.getSelectedSensorPosition();
  }
/**
 * Returns current arm position in degrees.
 * @return Current arm position from falcons, calculated to change to degrees.
  */
  public double getArmMotorPos(){
    
    double test =  Conversions.falconToDegrees(getbackArmMotorPosition(),(56));
    return test;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Position", getArmPosition() );
    SmartDashboard.putNumber("Arm Motor Position", getArmMotorPos());
    SmartDashboard.putNumber("Arm Motor 1 Current", leftArm.getStatorCurrent());
    SmartDashboard.putNumber("Arm Motor 2 Current", rightArm.getStatorCurrent());
  }
} 
