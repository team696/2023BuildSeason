// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ArmSub extends SubsystemBase {
  public TalonFX leftArm;
  public TalonFX rightArm; 

  public TalonFX telescopeArm;

  public TalonFX gripperJointFalcon;

  public CANSparkMax gripperJointNeo;
  public SparkMaxPIDController gripperJointPID;
  // public SparkMaxAbsoluteEncoder encoder;
  public RelativeEncoder jointEncoder;
  public PIDController armPID;
  public PIDController jointPID;
  public PIDController telescopePID;
  public CANCoder testCanCoder;

  private SupplyCurrentLimitConfiguration limit;

  public double rotMaxSpeedFor;
  public double rotMaxSpeedRev;

  public double telMaxSpeedFor;
  public double telMaxSpeedRev;
  


  /** Creates a new ArmSub. */
  public ArmSub() {

  
    testCanCoder = new CANCoder(14, "Karen");
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    testCanCoder.configSensorDirection(true);
    // testCanCoder.setPosition(0);
    testCanCoder.configMagnetOffset(-12);
    

    rotMaxSpeedFor = 1;
    rotMaxSpeedRev = 0.7;

    limit = new SupplyCurrentLimitConfiguration(true, 20, 20, 0);

    // gripperJointEncoder = new CANSparkMax(55, MotorType  .kBrushless);
    // encoder = gripperJointEncoder.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    gripperJointNeo = new CANSparkMax(45, MotorType.kBrushless);
    gripperJointNeo.clearFaults();
    gripperJointNeo.restoreFactoryDefaults();
    gripperJointNeo.getEncoder().setPosition(0);
    gripperJointNeo.setIdleMode(IdleMode.kBrake); 
    gripperJointNeo.setSmartCurrentLimit(30);
    gripperJointNeo.setInverted(true);
    // gripperJointNeo.getEncoder().setInverted(true);

    jointEncoder = gripperJointNeo.getEncoder();

    gripperJointPID = gripperJointNeo.getPIDController();
    gripperJointPID.setP(0.03);
    gripperJointPID.setI(0);
    gripperJointPID.setD(0);
    gripperJointPID.setFeedbackDevice(gripperJointNeo.getEncoder());
    gripperJointPID.setSmartMotionMaxAccel(5, 1);


    armPID = new PIDController(0.012, 0.0001, 0.0007);
    // armPID = new PIDController(0.012, 0.000, 0.000);

    armPID.setTolerance(0.5);
    armPID.enableContinuousInput(0, 360);
    
    // armPID.setIntegratorRange(1, 0.1);
    
    // pidLoop.

    leftArm = new WPI_TalonFX(20, "Karen");
    rightArm = new WPI_TalonFX(21, "Karen");
    telescopeArm = new WPI_TalonFX(22, "Karen");
    gripperJointFalcon = new WPI_TalonFX(40, "Karen");

    leftArm.configFactoryDefault();
      leftArm.setNeutralMode(NeutralMode.Brake);
      leftArm.configPeakOutputForward(rotMaxSpeedFor);
      leftArm.configPeakOutputReverse(-rotMaxSpeedRev);
      leftArm.config_kP(0, 0.05);
      leftArm.config_kI(0, 0.0);
      leftArm.config_kD(0, 0.0);
      leftArm.config_kF(0, 0.06);
      leftArm.setSensorPhase(true);
      leftArm.setInverted(InvertType.InvertMotorOutput);
      leftArm.configRemoteFeedbackFilter(testCanCoder, 0);
      leftArm.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
 


    rightArm.configFactoryDefault();
      rightArm.configPeakOutputForward(rotMaxSpeedFor);
      rightArm.configPeakOutputReverse(-rotMaxSpeedRev);
      rightArm.setSensorPhase(true);
      rightArm.setInverted(InvertType.FollowMaster);
      rightArm.config_kP(0, 0.05);
      rightArm.config_kI(0, 0.0);
      rightArm.config_kD(0, 0.0);
      rightArm.config_kF(0, 0.06);
      rightArm.setNeutralMode(NeutralMode.Brake);
      rightArm.follow(leftArm);

      telMaxSpeedFor = 1;
      telMaxSpeedRev = 1;
    
      telescopeArm.configFactoryDefault();
      telescopeArm.configPeakOutputForward(telMaxSpeedFor);
      telescopeArm.configPeakOutputReverse(-telMaxSpeedRev);
      telescopeArm.setSensorPhase(true);
      telescopeArm.setInverted(InvertType.None);
      telescopeArm.config_kP(0, 0.5);
      telescopeArm.config_kI(0, 0.0);
      telescopeArm.config_kD(0, 0.0);
      telescopeArm.config_kF(0, 0.0);
      telescopeArm.setNeutralMode(NeutralMode.Brake);
      telescopeArm.configNeutralDeadband(0.1);
      telescopeArm.configMotionAcceleration(100000);
      telescopeArm.configMotionCruiseVelocity(10000);
      telescopeArm.setSelectedSensorPosition(0);
      telescopeArm.configSupplyCurrentLimit(limit);

      gripperJointFalcon.configFactoryDefault();
      gripperJointFalcon.configPeakOutputForward(telMaxSpeedFor);
      gripperJointFalcon.configPeakOutputReverse(-telMaxSpeedRev);
      gripperJointFalcon.setSensorPhase(true);
      gripperJointFalcon.setInverted(InvertType.None);
      gripperJointFalcon.config_kP(0, 0.01);
      gripperJointFalcon.config_kI(0, 0.0);
      gripperJointFalcon.config_kD(0, 0.0);
      gripperJointFalcon.config_kF(0, 0.0);
      gripperJointFalcon.setNeutralMode(NeutralMode.Brake);
      gripperJointFalcon.configNeutralDeadband(0.1);
      gripperJointFalcon.configMotionAcceleration(100000);
      gripperJointFalcon.configMotionCruiseVelocity(10000);
      gripperJointFalcon.setSelectedSensorPosition(0);
      gripperJointFalcon.configSupplyCurrentLimit(limit);
      
    


  }

  public void ArmBrakeMode(NeutralMode mode){
    telescopeArm.setNeutralMode(mode);
    rightArm.setNeutralMode(mode);
    leftArm.setNeutralMode(mode);
  }
 

 public void moveRotArmMotionMagic(double degrees){
  // TODO degrees to falcon readout
  // double setpoint = degrees *3;
  // leftArm.set(TalonFXControlMode.MotionMagic, setpoint);
  leftArm.set(ControlMode.PercentOutput ,armPID.calculate(testCanCoder.getAbsolutePosition(), degrees));

 }

//  public void homeArmRotPos(){
//   testCanCoder.setPosition(0);
  
//  }
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
 * Moves the arm telescope using percent control.
 * @param percent 
  */
public void extendArmPercentOutput(double percent){
  telescopeArm.set(TalonFXControlMode.PercentOutput, percent);
}

public void extendArmPosition(double position){
  // telescopeArm.set(TalonFXControlMode.Position, position);
  telescopeArm.set(TalonFXControlMode.MotionMagic, position);
}

public void gripperJointPosition(double position){
  gripperJointFalcon.set(TalonFXControlMode.MotionMagic, position);
}

public void manualJointControl(double percent ){
  // gripperJointNeo.set(percent);
  gripperJointFalcon.set(TalonFXControlMode.PercentOutput, percent);

}

public double getTelescopePos(){
  return telescopeArm.getSelectedSensorPosition();
}
public void homeTelescopePosition(){
  telescopeArm.setSelectedSensorPosition(0);
}

public void homeRotArmPos(){
  // TODO THING
  leftArm.setSelectedSensorPosition(testCanCoder.getPosition() /30);
}

public void homeJointPos(){
  // gripperJointNeo.getEncoder().setPosition(0);
  gripperJointFalcon.setSelectedSensorPosition(0);
}

/**
 * Moves the arm to a dedicated degree.
 * @param position Desired arm position in degrees
 */
  public void moveArmPosition(double position){
    leftArm.set(ControlMode.PercentOutput ,armPID.calculate(testCanCoder.getAbsolutePosition(), position));
  }
/**
 * Moves the arm to a dedicated preset positions
 * @param position The dedicated position enum, current options are GROUND_PICKUP, STOWED, GROUND_SCORE, MID_SCORE, HIGH_SCORE, and SHELF_PICKUP
  */
  public void armRotPresetPositions(GlobalVariables.ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForConePickup);
        GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForConePickup;
      }
      else{
        moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForCubePickup);
        GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForCubePickup;

      }
      break;

      case STOWED_ADAPTIVE:
      moveRotArmMotionMagic(Constants.ArmRotationValues.armRotStow);
      GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotStow;

      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForLowCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForLowCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForLowCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForLowCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevLowCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevLowCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevLowCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevLowCube;

        }
      }
      break;

      case SHELF_PICKUP_ADAPTIVE:

      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForShelfCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForShelfCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForShelfCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForShelfCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevShelfCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevShelfCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevShelfCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevShelfCube;

        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
        if(GlobalVariables.robotDirection){
          if(GlobalVariables.gamePiece == 0){
            moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForMidCone);
            GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForMidCone;

          }
          else{
            moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForMidCube);
            GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForMidCube;

          }
        }
        else{
          if(GlobalVariables.gamePiece == 0){
            moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevMidCone);
            GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevMidCone;

          }
          else{
            moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevMidCube);
            GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevMidCube;

          }
        }
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForHighCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForHighCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotForHighCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotForHighCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmMotionMagic( Constants.ArmRotationValues.armRotRevHighCone);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevHighCone;

        }
        else{
          moveRotArmMotionMagic(Constants.ArmRotationValues.armRotRevHighCube);
          GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotRevHighCube;

        }
      }
      break;

    

      default:
      moveRotArmMotionMagic(Constants.ArmRotationValues.armRotStow);
      GlobalVariables.armRotGoal = Constants.ArmRotationValues.armRotStow;

      break;
    }
  }

  public void armExtendPresetPositions(GlobalVariables.ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        extendArmPosition(Constants.ArmExtendValues.armExtendConePickup);
        GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendConePickup;
      }
      else{
        extendArmPosition(Constants.ArmExtendValues.armExtendCubePickup);
        GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendCubePickup;


      }
      break;

      case STOWED_ADAPTIVE:
      extendArmPosition(Constants.ArmExtendValues.armExtendStow);
      GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendStow;

      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendForLowCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForLowCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendForLowCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForLowCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendRevLowCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevLowCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendRevLowCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevLowCube;


        }
      }
     
      break;

      case SHELF_PICKUP_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendForShelfCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForShelfCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendForShelfCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForShelfCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendRevShelfCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevShelfCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendRevShelfCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevShelfCube;


        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendForMidCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForMidCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendForMidCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendForMidCube;

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendRevMidCone);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevMidCone;

        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendRevMidCube);
          GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendRevMidCube;


        }
      }
     
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendForHighCone);
        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendForHighCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.ArmExtendValues.armExtendRevHighCone);
        }
        else{
          extendArmPosition(Constants.ArmExtendValues.armExtendRevHighCube);

        }
      }
     
      break;

    

      default:
      extendArmPosition(Constants.ArmExtendValues.armExtendStow);
      GlobalVariables.armExtendGoal = Constants.ArmExtendValues.armExtendStow;


      break;
    }
  }

  public void jointRotPresetPositions(GlobalVariables.ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        extendArmPosition(Constants.JointRotationValues.JointRotConePickup);
      }
      else{
        extendArmPosition(Constants.JointRotationValues.JointRotCubePickup);

      }
      break;

      case STOWED_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        extendArmPosition(Constants.JointRotationValues.JointRotStowCone);
      }
      else{
        extendArmPosition(Constants.JointRotationValues.JointRotStowCube);

      }
      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotForLowCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotForLowCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotRevLowCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotRevLowCube);

        }
      }
     
      break;

      case SHELF_PICKUP_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotForShelfCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotForShelfCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotRevShelfCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotRevShelfCube);

        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotForMidCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotForMidCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotRevMidCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotRevMidCube);

        }
      }
     
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(GlobalVariables.robotDirection){
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotForHighCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotForHighCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          extendArmPosition(Constants.JointRotationValues.JointRotRevHighCone);
        }
        else{
          extendArmPosition(Constants.JointRotationValues.JointRotRevHighCube);

        }
      }
     
      break;

    

      default:
      if(GlobalVariables.gamePiece == 0){
        extendArmPosition(Constants.JointRotationValues.JointRotStowCone);
      }
      else{
        extendArmPosition(Constants.JointRotationValues.JointRotStowCube);

      }
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

  public double getRotArmPos(){
    // TODO make this gearbox to degrees
    return leftArm.getSelectedSensorPosition() /3;
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

  public double getJointPos(){
    // return jointEncoder.getPosition();
    return gripperJointFalcon.getSelectedSensorPosition();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("PID OUTPUT ", leftArm.getMotorOutputPercent());
    SmartDashboard.putNumber("PID ERROR ", armPID.getPositionError());
    SmartDashboard.putNumber("Arm Encoder Position", getArmPosition() );
    SmartDashboard.putNumber("Arm Motor Position", getArmMotorPos());
    SmartDashboard.putNumber("Arm Motor 1 Current", leftArm.getStatorCurrent());
    SmartDashboard.putNumber("Arm Motor 2 Current", rightArm.getStatorCurrent());
    SmartDashboard.putNumber("Telescope Position", telescopeArm.getSelectedSensorPosition());
    SmartDashboard.putNumber("Telescope Speed?", telescopeArm.getSupplyCurrent());
    SmartDashboard.putNumber("Joint Position ", getJointPos() );
    SmartDashboard.putNumber("Joint Motor Current ", gripperJointNeo.getOutputCurrent());
  }
} 
