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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ArmPositions;

public class ArmSub extends SubsystemBase {
  public TalonFX leftArm;
  public TalonFX rightArm; 

  public TalonFX telescopeArm;

  public TalonFX gripperJointFalcon;


  public ProfiledPIDController armPID;
  public PIDController jointPID;
  public PIDController telescopePID;
  public CANCoder testCanCoder;

  public PIDController slowArmPID;

  private SupplyCurrentLimitConfiguration limit;

  public double rotMaxSpeedFor;
  public double rotMaxSpeedRev;

  public double telMaxSpeedFor;
  public double telMaxSpeedRev;
  
  public double multiplier;

  public double armExtendGoal = 0;
  public double armRotGoal = 100;

  public boolean robotDirection = false;

  /** Creates a new ArmSub. */
  public ArmSub() {
    testCanCoder = new CANCoder(14, "Karen");
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    testCanCoder.configSensorDirection(true);
    testCanCoder.configMagnetOffset(150);

    multiplier = 0.677;
    

    rotMaxSpeedFor = 1;
    rotMaxSpeedRev = 0.6;

    limit = new SupplyCurrentLimitConfiguration(true, 30, 30, 0);

    armPID = new ProfiledPIDController(0.01, 0.000, 0.000, new TrapezoidProfile.Constraints(1, 1));
    armPID.setTolerance(0.1);
    armPID.enableContinuousInput(0, 360);

    slowArmPID = new PIDController(0.007, 0.000, 0.000);
    slowArmPID.setTolerance(0.1);
    slowArmPID.enableContinuousInput(0, 360);

    
    

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
      telescopeArm.config_kP(0, 0.7);
      telescopeArm.config_kI(0, 0.0);
      telescopeArm.config_kD(0, 0.0);
      telescopeArm.config_kF(0, 0.0);
      telescopeArm.setNeutralMode(NeutralMode.Brake);
      telescopeArm.configNeutralDeadband(0.1);
      telescopeArm.configMotionAcceleration(45000);
      telescopeArm.configMotionCruiseVelocity(300000);
      telescopeArm.setSelectedSensorPosition(0);
      telescopeArm.configSupplyCurrentLimit(limit);

      gripperJointFalcon.configFactoryDefault();
      gripperJointFalcon.configPeakOutputForward(telMaxSpeedFor);
      gripperJointFalcon.configPeakOutputReverse(-telMaxSpeedRev);
      gripperJointFalcon.setSensorPhase(true);
      gripperJointFalcon.setInverted(InvertType.InvertMotorOutput);
      gripperJointFalcon.config_kP(0, 0.6);
      gripperJointFalcon.config_kI(0, 0.0);
      gripperJointFalcon.config_kD(0, 0.0);
      gripperJointFalcon.config_kF(0, 0.0);
      gripperJointFalcon.setNeutralMode(NeutralMode.Brake);
      gripperJointFalcon.configNeutralDeadband(0.001);
      gripperJointFalcon.configMotionAcceleration(30000);
      gripperJointFalcon.configMotionCruiseVelocity(40000);
      gripperJointFalcon.setSelectedSensorPosition(0);
      gripperJointFalcon.configAllowableClosedloopError(0, 10);
      
      gripperJointFalcon.configSupplyCurrentLimit(limit);

      SmartDashboard.putData(this);
  }

  public void ArmBrakeMode(NeutralMode mode){
    telescopeArm.setNeutralMode(mode);
    rightArm.setNeutralMode(mode);
    leftArm.setNeutralMode(mode);
  }
 

 public void moveRotArmPosition(double degrees){
  leftArm.set(ControlMode.PercentOutput, armPID.calculate(testCanCoder.getAbsolutePosition(), degrees));
  armRotGoal = degrees;
 }

 public void moveRotArmPositionSlow(double degrees){
  leftArm.set(ControlMode.PercentOutput, slowArmPID.calculate(testCanCoder.getAbsolutePosition(), degrees));
 }


/**
 * Moves the arm by percent output.
 * @param percent 0.0 - 1.0.
  */
  public void moveRotArmPercentOutput(double percent){
    leftArm.set(TalonFXControlMode.PercentOutput, percent);
  }

   
/**
 * Moves the arm telescope using percent control.
 * @param percent 
  */
public void moveTelescopeArmPercentOutput(double percent){
  telescopeArm.set(TalonFXControlMode.PercentOutput, percent);
}

public void moveTelescopeArmPosition(double position){
  telescopeArm.set(TalonFXControlMode.MotionMagic, position);
  armExtendGoal = position;
}

public void moveGripperJointPosition(double position){
  gripperJointFalcon.set(TalonFXControlMode.MotionMagic, position);
}



public void moveGripperJointPercentOutput(double percent ){
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
  leftArm.setSelectedSensorPosition(testCanCoder.getPosition() /30);
}

public void homeGripperJointPos(){
  // gripperJointNeo.getEncoder().setPosition(0);
  gripperJointFalcon.setSelectedSensorPosition(0);
}

/**
 * Moves the arm to a dedicated degree.
 * @param position Desired arm position in degrees
 */
 
/**
 * Moves the arm to a dedicated preset positions
 * @param position The dedicated position enum, current options are GROUND_PICKUP, STOWED, GROUND_SCORE, MID_SCORE, HIGH_SCORE, and SHELF_PICKUP
  */
  public void armRotPresetPositions(ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        moveRotArmPosition(Constants.ArmRotationValues.armRotForConePickup);
      }
      else{
        moveRotArmPosition(Constants.ArmRotationValues.armRotForCubePickup);

      }
      break;

      case STOWED_ADAPTIVE:
      if(robotDirection){
      moveRotArmPositionSlow(Constants.ArmRotationValues.armRotStow);
      }
      else{
        moveRotArmPosition(Constants.ArmRotationValues.armRotStow);
      }
      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition(Constants.ArmRotationValues.armRotForLowCone);

        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotForLowCube);

        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition(Constants.ArmRotationValues.armRotRevLowCone);
        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotRevLowCube);
        }
      }
      break;

      case SHELF_PICKUP_ADAPTIVE:

      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition(Constants.ArmRotationValues.armRotForShelfCone);
        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotForShelfCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition(Constants.ArmRotationValues.armRotRevShelfCone);
        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotRevShelfCube);
        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
        if(robotDirection){
          if(GlobalVariables.gamePiece == 0){
            moveRotArmPosition(Constants.ArmRotationValues.armRotForMidCone);
          }
          else{
            moveRotArmPosition(Constants.ArmRotationValues.armRotForMidCube);
          }
        }
        else{
          if(GlobalVariables.gamePiece == 0){
            moveRotArmPosition(Constants.ArmRotationValues.armRotRevMidCone);
          }
          else{
            moveRotArmPosition(Constants.ArmRotationValues.armRotRevMidCube);
          }
        }
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition(Constants.ArmRotationValues.armRotForHighCone);
        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotForHighCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveRotArmPosition( Constants.ArmRotationValues.armRotRevHighCone);
        }
        else{
          moveRotArmPosition(Constants.ArmRotationValues.armRotRevHighCube);
        }
      }
      break;

      case FRAME_PERIMETER:
        moveRotArmPosition(Constants.ArmRotationValues.framePerimeter);
      break;

      default:
      moveRotArmPosition(Constants.ArmRotationValues.armRotStow);
      break;
    }
  }

  public void armExtendPresetPositions(ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendConePickup);
      }
      else{
        moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendCubePickup);

      }
      break;

      case STOWED_ADAPTIVE:
      moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendStow);
      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForLowCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForLowCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevLowCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevLowCube);
        }
      }
     
      break;

      case SHELF_PICKUP_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForShelfCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForShelfCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevShelfCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevShelfCube);
        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForMidCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForMidCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevMidCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevMidCube);

        }
      }
     
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForHighCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendForHighCube);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevHighCone);
        }
        else{
          moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendRevHighCube);
        }
      }
     
      break;

      case FRAME_PERIMETER:
        moveTelescopeArmPosition(Constants.ArmExtendValues.framePerimeter);
              break;

      default:
      moveTelescopeArmPosition(Constants.ArmExtendValues.armExtendStow);

      break;
    }
  }

  public void jointRotPresetPositions(ArmPositions position){
    switch(position){

      case GROUND_PICKUP_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        moveGripperJointPosition(Constants.JointRotationValues.JointRotConePickup * multiplier);
      }
      else{
        moveGripperJointPosition(Constants.JointRotationValues.JointRotCubePickup* multiplier);

      }
      break;

      case STOWED_ADAPTIVE:
      if(GlobalVariables.gamePiece == 0){
        moveGripperJointPosition(Constants.JointRotationValues.JointRotStowCone* multiplier);
      }
      else{
        moveGripperJointPosition(Constants.JointRotationValues.JointRotStowCube* multiplier);

      }
      break;

      case GROUND_SCORE_ADAPTIVE: 
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForLowCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForLowCube* multiplier);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevLowCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevLowCube* multiplier);

        }
      }
     
      break;

      case SHELF_PICKUP_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForShelfCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForShelfCube* multiplier);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevShelfCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevShelfCube* multiplier);

        }
      }
      break;

      case MID_SCORE_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForMidCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForMidCube* multiplier);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevMidCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevMidCube* multiplier);

        }
      }
     
      break;

      case HIGH_SCORE_ADAPTIVE:
      if(robotDirection){
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForHighCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotForHighCube* multiplier);
        }
      }
      else{
        if(GlobalVariables.gamePiece == 0){
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevHighCone* multiplier);
        }
        else{
          moveGripperJointPosition(Constants.JointRotationValues.JointRotRevHighCube* multiplier);

        }
      }
     
      break;

      case FRAME_PERIMETER:
        moveGripperJointPosition(Constants.JointRotationValues.framePerimeter * multiplier);
      break;

      default:
      if(GlobalVariables.gamePiece == 0){
        moveGripperJointPosition(Constants.JointRotationValues.JointRotStowCone* multiplier);
      }
      else{
        moveGripperJointPosition(Constants.JointRotationValues.JointRotStowCube* multiplier);

      }
      break;
    }
  }
  
/**
 * Returns current arm position
 * @return CANcoder arm position in degrees.
  */
  public double getArmEncoderPosition(){
    return testCanCoder.getAbsolutePosition();
  }

  public double getRotArmPos(){
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
    return Conversions.falconToDegrees(getbackArmMotorPosition(),(56));
  }

  public double getGripperJointPos(){
    return gripperJointFalcon.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Arm Encoder Position", ()->getArmEncoderPosition(), null);
    builder.addDoubleProperty("Arm Motor Position", ()->getArmMotorPos(), null);
    builder.addDoubleProperty("Arm Motor Velocity", ()->leftArm.getSelectedSensorVelocity(), null);
    builder.addDoubleProperty("Arm Motor Set Speed", ()-> leftArm.getMotorOutputPercent(), null);
    builder.addDoubleProperty("Telescope Position", ()->telescopeArm.getSelectedSensorPosition(), null);
    builder.addDoubleProperty("Telescope Current", ()->telescopeArm.getSupplyCurrent(), null);
    builder.addDoubleProperty("Joint Position", ()->getGripperJointPos(), null);
    builder.addDoubleProperty("Joint Speed", ()->gripperJointFalcon.getSelectedSensorVelocity(), null);

    SmartDashboard.putData("ArmPID", armPID);
  }
} 
