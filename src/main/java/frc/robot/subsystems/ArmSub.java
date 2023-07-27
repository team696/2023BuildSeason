// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
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
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.ArmPositions;

public class ArmSub extends SubsystemBase {
  private TalonFX leftArm;
  private TalonFX rightArm; 

  private TalonFX telescopeArm;

  private TalonFX gripperJointFalcon;


  private PIDController armPID;
  private CANCoder testCanCoder;

  private PIDController slowArmPID;

  private SupplyCurrentLimitConfiguration limit;

  private double rotMaxSpeedFor = 1;
  private double rotMaxSpeedRev = 0.6;

  private double telMaxSpeedFor = 1;
  private double telMaxSpeedRev = 1;
  
  private final double multiplier = 0.667;

  public double armExtendGoal = 0;
  public double armRotGoal = 100;

  public boolean robotDirection = false;

  public static int gamePiece = 0;

  private double testFF = 0;
                              //frd/rev, cone/cube
  private HashMap<ArmPositions, double[][]> ShoulderPos = new HashMap<>();
  private void addPostoShoulder(ArmPositions pos, double w, double x, double y, double z) {
    ShoulderPos.put(pos, new double[][] { 
      {w,x}, 
      {y,z}
    });
  }

  private HashMap<ArmPositions, double[][]> ExtendPos = new HashMap<>();
  private void addPostoExtend(ArmPositions pos, double w, double x, double y, double z) {
    ExtendPos.put(pos, new double[][] { 
      {w,x}, 
      {y,z}
    });
  }

  private HashMap<ArmPositions, double[][]> JointPos = new HashMap<>();
  private void addPostoJoint(ArmPositions pos, double w, double x, double y, double z) {
    JointPos.put(pos, new double[][] { 
      {w,x}, 
      {y,z}
    });
  }
  /** Creates a new ArmSub. */
  public ArmSub() {
    testCanCoder = new CANCoder(14, "Karen");
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    testCanCoder.configSensorDirection(true);
    testCanCoder.configMagnetOffset(150);

    limit = new SupplyCurrentLimitConfiguration(true, 30, 30, 0);

    armPID = new PIDController(0.01, 0.000, 0.000);
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

      addPostoShoulder(ArmPositions.GROUND_PICKUP_ADAPTIVE, Constants.ArmRotationValues.armRotForConePickup, Constants.ArmRotationValues.armRotForCubePickup, Constants.ArmRotationValues.armRotForConePickup, Constants.ArmRotationValues.armRotForCubePickup);
      addPostoShoulder(ArmPositions.STOWED_ADAPTIVE, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow);
      addPostoShoulder(ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.ArmRotationValues.armRotForLowCone, Constants.ArmRotationValues.armRotForLowCube,Constants.ArmRotationValues.armRotRevLowCone,Constants.ArmRotationValues.armRotRevLowCube);
      addPostoShoulder(ArmPositions.SHELF_PICKUP_ADAPTIVE,  Constants.ArmRotationValues.armRotForShelfCone, Constants.ArmRotationValues.armRotForShelfCube, Constants.ArmRotationValues.armRotRevShelfCone, Constants.ArmRotationValues.armRotRevShelfCube);
      addPostoShoulder(ArmPositions.MID_SCORE_ADAPTIVE ,Constants.ArmRotationValues.armRotForMidCone ,Constants.ArmRotationValues.armRotForMidCube ,Constants.ArmRotationValues.armRotRevMidCone ,Constants.ArmRotationValues.armRotRevMidCube );
      addPostoShoulder(ArmPositions.HIGH_SCORE_ADAPTIVE ,Constants.ArmRotationValues.armRotForHighCone, Constants.ArmRotationValues.armRotForHighCube, Constants.ArmRotationValues.armRotRevHighCone, Constants.ArmRotationValues.armRotRevHighCube);
      addPostoShoulder(ArmPositions.FRAME_PERIMETER ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter );  

      addPostoExtend  (ArmPositions.GROUND_PICKUP_ADAPTIVE, Constants.ArmExtendValues.armExtendConePickup, Constants.ArmExtendValues.armExtendCubePickup, Constants.ArmExtendValues.armExtendConePickup, Constants.ArmExtendValues.armExtendCubePickup);
      addPostoExtend  (ArmPositions.STOWED_ADAPTIVE, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow);
      addPostoExtend  (ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.ArmExtendValues.armExtendForLowCone, Constants.ArmExtendValues.armExtendForLowCube, Constants.ArmExtendValues.armExtendRevLowCone, Constants.ArmExtendValues.armExtendRevLowCube);
      addPostoExtend  (ArmPositions.SHELF_PICKUP_ADAPTIVE, Constants.ArmExtendValues.armExtendForShelfCone, Constants.ArmExtendValues.armExtendForShelfCube, Constants.ArmExtendValues.armExtendRevShelfCone, Constants.ArmExtendValues.armExtendRevShelfCube);
      addPostoExtend  (ArmPositions.MID_SCORE_ADAPTIVE, Constants.ArmExtendValues.armExtendForMidCone, Constants.ArmExtendValues.armExtendForMidCube, Constants.ArmExtendValues.armExtendRevMidCone, Constants.ArmExtendValues.armExtendRevMidCube);
      addPostoExtend  (ArmPositions.HIGH_SCORE_ADAPTIVE, Constants.ArmExtendValues.armExtendForHighCone, Constants.ArmExtendValues.armExtendForHighCube, Constants.ArmExtendValues.armExtendRevHighCone, Constants.ArmExtendValues.armExtendRevHighCube);
      addPostoExtend  (ArmPositions.FRAME_PERIMETER, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter);
      
      addPostoJoint   (ArmPositions.GROUND_PICKUP_ADAPTIVE, Constants.JointRotationValues.JointRotConePickup, Constants.JointRotationValues.JointRotCubePickup, Constants.JointRotationValues.JointRotConePickup, Constants.JointRotationValues.JointRotCubePickup);
      addPostoJoint   (ArmPositions.STOWED_ADAPTIVE, Constants.JointRotationValues.JointRotStowCone, Constants.JointRotationValues.JointRotStowCube, Constants.JointRotationValues.JointRotStowCone, Constants.JointRotationValues.JointRotStowCube);
      addPostoJoint   (ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.JointRotationValues.JointRotForLowCone, Constants.JointRotationValues.JointRotForLowCube, Constants.JointRotationValues.JointRotRevLowCone, Constants.JointRotationValues.JointRotRevLowCube);
      addPostoJoint   (ArmPositions.SHELF_PICKUP_ADAPTIVE, Constants.JointRotationValues.JointRotForShelfCone, Constants.JointRotationValues.JointRotForShelfCube, Constants.JointRotationValues.JointRotRevShelfCone, Constants.JointRotationValues.JointRotRevShelfCube);
      addPostoJoint   (ArmPositions.MID_SCORE_ADAPTIVE, Constants.JointRotationValues.JointRotForMidCone, Constants.JointRotationValues.JointRotForMidCube, Constants.JointRotationValues.JointRotRevMidCone, Constants.JointRotationValues.JointRotRevMidCube);
      addPostoJoint   (ArmPositions.HIGH_SCORE_ADAPTIVE, Constants.JointRotationValues.JointRotForHighCone, Constants.JointRotationValues.JointRotForHighCube, Constants.JointRotationValues.JointRotRevHighCone, Constants.JointRotationValues.JointRotRevHighCube);
      addPostoJoint   (ArmPositions.FRAME_PERIMETER, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter);
  
      SmartDashboard.putData(this);
  }

  public void ArmBrakeMode(NeutralMode mode){
    telescopeArm.setNeutralMode(mode);
    rightArm.setNeutralMode(mode);
    leftArm.setNeutralMode(mode);
  }
 

 public void moveRotArmPosition(double degrees){

  final int kMeasuredPosHorizontal = 34;  // HORIZONTAL VALUE
  final double kTicksPerDegree = 360 / 360; 
  double currentPos = getArmEncoderPosition();
  double degree = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; 
  double radians = java.lang.Math.toRadians(degree);
  double cosineScalar = java.lang.Math.cos(radians);
  
  final double maxGravityFFRet = 0.07; //FORCE REQUIRED TO KEEP ARM AT HORIZONTAL WITH ARM RETRACTED, DETERMINE WITH testFF
  final double maxGravityFFExt = 0.1;  // ^ BUT EXTENDED                  //MAX EXTENSION VALUE
  double endFF = maxGravityFFRet + (telescopeArm.getSelectedSensorPosition() / 6969 * (maxGravityFFExt - maxGravityFFRet)) * cosineScalar;

  leftArm.set(ControlMode.PercentOutput, armPID.calculate(testCanCoder.getAbsolutePosition(), degrees), DemandType.ArbitraryFeedForward, testFF);
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
  gripperJointFalcon.set(TalonFXControlMode.MotionMagic, position * multiplier);
}



public void moveGripperJointPercentOutput(double percent ){
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
    moveRotArmPosition(ShoulderPos.get(position)[robotDirection ? 1 : 0][gamePiece]+3);
  }

  public void armExtendPresetPositions(ArmPositions position){
    moveTelescopeArmPosition(ExtendPos.get(position)[robotDirection ? 1 : 0][gamePiece]);
  }

  public void jointRotPresetPositions(ArmPositions position){
    moveGripperJointPosition(JointPos.get(position)[robotDirection ? 1 : 0][gamePiece] * multiplier);
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
    builder.addBooleanProperty("Arm Encoder Status", ()->(testCanCoder.getMagnetFieldStrength().value == 3), null);
    builder.addDoubleProperty("Arm Motor Position", ()->getArmMotorPos(), null);
    builder.addDoubleProperty("Arm Motor Velocity", ()->leftArm.getSelectedSensorVelocity(), null);
    builder.addDoubleProperty("Arm Motor Set Speed", ()-> leftArm.getMotorOutputPercent(), null);
    builder.addDoubleProperty("Telescope Position", ()->telescopeArm.getSelectedSensorPosition(), null);
    builder.addDoubleProperty("Telescope Current", ()->telescopeArm.getSupplyCurrent(), null);
    builder.addDoubleProperty("Joint Position", ()->getGripperJointPos(), null);
    builder.addDoubleProperty("Joint Speed", ()->gripperJointFalcon.getSelectedSensorVelocity(), null);

    SmartDashboard.putData("ArmPID", armPID);
    builder.addDoubleProperty("Arm FF", ()->testFF, (t)->testFF=t);
    builder.setActuator(true);
  }
} 
