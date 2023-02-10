// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ArmSub extends SubsystemBase {
  public TalonFX leftArm;
  public TalonFX rightArm; 
  public PIDController pidLoop;
  // public Counter lampreyEncoder;
  public TalonSRX encorderTalon;
  public CANCoder testCanCoder;
  public TalonSRXFeedbackDevice encoder;
  // DigitalInput test
   RemoteSensorSource lampreyTest;
  /** Creates a new ArmSub. */
  public ArmSub() {
    // lampreyEncoder = new  Counter(Counter.Mode.kSemiperiod);
    // lampreyEncoder.setUpSource(1);
    encorderTalon = new TalonSRX(55);
    encorderTalon.configFactoryDefault();
    testCanCoder = new CANCoder(14, "Abu");
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // encorderTalon.configRemoteFeedbackFilter(encorderTalon, 0);
    // encorderTalon.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0);
    
    // encorderTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    pidLoop = new PIDController(0.01, 0.000, 0.005);
    leftArm = new WPI_TalonFX(20, "Abu");
    rightArm = new WPI_TalonFX(21, "Abu");
    leftArm.configFactoryDefault();
    leftArm.setNeutralMode(NeutralMode.Brake);
    leftArm.configPeakOutputForward(1);
    leftArm.configPeakOutputReverse(-1);
    leftArm.config_kP(0, 0.05);
    leftArm.config_kI(0, 0.0);
    leftArm.config_kD(0, 0.0);
    leftArm.config_kF(0, 0.06);
    leftArm.setSensorPhase(true);
    leftArm.setInverted(InvertType.InvertMotorOutput);
    leftArm.configRemoteFeedbackFilter(testCanCoder, 0);
    leftArm.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    // leftArm.configClosedloopRamp(0.2);
    // leftArm.configOpenloopRamp(0.2);


    rightArm.configFactoryDefault();
    rightArm.configPeakOutputForward(1);
    rightArm.configPeakOutputReverse(-1);
    rightArm.setSensorPhase(true);
    rightArm.setInverted(InvertType.FollowMaster);
    rightArm.config_kP(0, 0.05);
    rightArm.config_kI(0, 0.0);
    rightArm.config_kD(0, 0.0);
    rightArm.config_kF(0, 0.06);
    rightArm.setNeutralMode(NeutralMode.Brake);
    // rightArm.configClosedloopRamp(0.2);
    // rightArm.configOpenloopRamp(0.2);

    rightArm.follow(leftArm);
    
    // resetToAbsolute();

  }
/**
 * Moves the arm by percent output.
 * @param percent 0.0 - 1.0.
  */
  public void moveArmPercentOutput(double percent){
    leftArm.set(TalonFXControlMode.PercentOutput, percent);
  }

  private void resetToAbsolute(){
    double absolutePosition = Conversions.degreesToFalcon(testCanCoder.getAbsolutePosition(), (56));
    leftArm.setSelectedSensorPosition(absolutePosition);
}

  public void moveArmPosition(double position){
    // leftArm.set(ControlMode.Position, position);
    // leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(encorderTalon.getSelectedSensorPosition(), position));
    leftArm.set(TalonFXControlMode.Position, position);
  }

  public void armPresetPositions(GlobalVariables.ArmPositions position){
    switch(position){

      case GROUND_PICKUP:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.grndIntakePosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.grndIntakePosValue);

      
      break;

      case STOWED:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.stowedPosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.stowedPosValue);
      break;

      case GROUND_SCORE: 
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.grndScorePosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.grndScorePosValue);

      break;

      case MID_SCORE:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.midScorePosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.midScorePosValue);

      break;

      case HIGH_SCORE:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.highScorePosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.highScorePosValue);

      break;

      case SHELF_PICKUP:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.shelfIntakePosValue));
      // leftArm.set(TalonFXControlMode.Position, Constants.shelfIntakePosValue);


      break;
      default:
      leftArm.set(ControlMode.PercentOutput ,pidLoop.calculate(testCanCoder.getAbsolutePosition(), Constants.shelfIntakePosValue));

        // leftArm.set(TalonFXControlMode.Position, Constants.shelfIntakePosValue);
      break;
    }
  }

  public double getArmPosition(){
    return testCanCoder.getAbsolutePosition();
  }

  public double getbackArmMotorPosition(){
    return leftArm.getSelectedSensorPosition();
  }

  public double getArmMotorPos(){
    
    double test =  Conversions.falconToDegrees(getbackArmMotorPosition(),(56));
    return test;
  }


  @Override
  public void periodic() {
    System.out.print(testCanCoder.getAbsolutePosition());
    // System.out.print( (leftArm.getSelectedSensorPosition() /* * 0.00274658 */) /* * (360.0 / ((64) * 2048.0)) */ );
    // System.out.print(getArmMotorPos());
    SmartDashboard.putNumber("Arm Encoder Position", getArmPosition() );
    SmartDashboard.putNumber("Arm Motor Position", getArmMotorPos());
    SmartDashboard.putNumber("Arm Motor 1 Current", leftArm.getStatorCurrent());
    SmartDashboard.putNumber("Arm Motor 2 Current", rightArm.getStatorCurrent());
    // This method will be called once per scheduler run
  }
} 
