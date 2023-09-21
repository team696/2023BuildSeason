// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Constants.ArmPositions;

public class ArmSub extends SubsystemBase {
  public static final double MAX_EXTENSION = 53075;

  private TalonFX leftArm;
  private TalonFX rightArm; 
  private TalonFX middleArm;

  private TalonFX telescopeArm;

  private TalonFX gripperJointFalcon;


  private PIDController armPID;
  private CANCoder testCanCoder;

  private ProfiledPIDController pArmPID;
  private ArmFeedforward armFF;

  private SupplyCurrentLimitConfiguration limit;

  private double rotMaxSpeedFor = 1;
  private double rotMaxSpeedRev = 1;

  private double telMaxSpeedFor = 1;
  private double telMaxSpeedRev = 1;
  
  private final double multiplier = 0.667 * 32/17 * 4/5;

  public double armExtendGoal = 0;
  public double armRotGoal = 0;
  public double wristRotGoal = 0;

  public int robotDirection = 0;

  private double test = 0;

  public boolean hasReset = false; // SUPER IMPORTANT!!! MAKES ProfiledPIDController NOT SHIT THE BED on startup. ALWAYS USE WITH PROFILEDPIDCONTROLLER

  public static int gamePiece = 0;
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
    testCanCoder.configFactoryDefault(); 
    testCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    testCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    testCanCoder.configSensorDirection(true);
    testCanCoder.configMagnetOffset(26);
    
    limit = new SupplyCurrentLimitConfiguration(true, 30, 30, 0);

    armPID = new PIDController(0.007, 0.000, 0.0);
    armPID.setTolerance(0);
    //armPID.enableContinuousInput(0, 360);

    TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(360, 720); 
    pArmPID = new ProfiledPIDController(0.016, 0.000, 0.000, m_constraints,0.02);
    pArmPID.disableContinuousInput();

    armFF = new ArmFeedforward(gamePiece, 0.5, 1.04, 0.03);

    leftArm = new WPI_TalonFX(20, "Karen");
    rightArm = new WPI_TalonFX(21, "Karen");
    middleArm = new WPI_TalonFX(23, "Karen");
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
      //leftArm.configRemoteFeedbackFilter(testCanCoder, 0); // WE CALCULATE OUR OWN SHIT A DIFF WAY -> MAYBE EXPERIMENT W/ THIS NEXT TIME, MOTION MAGIC!
      //leftArm.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);

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

      middleArm.configFactoryDefault();
      middleArm.configPeakOutputForward(rotMaxSpeedFor);
      middleArm.configPeakOutputReverse(-rotMaxSpeedRev);
      middleArm.setSensorPhase(true);
      middleArm.setInverted(InvertType.FollowMaster);
      middleArm.config_kP(0, 0.05);
      middleArm.config_kI(0, 0.0);
      middleArm.config_kD(0, 0.0);
      middleArm.config_kF(0, 0.06);
      middleArm.setNeutralMode(NeutralMode.Brake);
      middleArm.follow(leftArm);
    
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
      // REMOVE ALL THESE CONSTANTS MAYBE???
      addPostoShoulder(ArmPositions.GROUND_PICKUP_ADAPTIVE, -11, -11, -11, -11);
      addPostoShoulder(ArmPositions.STOWED_ADAPTIVE, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow, Constants.ArmRotationValues.armRotStow);
      addPostoShoulder(ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.ArmRotationValues.armRotForLowCone, 5,Constants.ArmRotationValues.armRotRevLowCone,Constants.ArmRotationValues.armRotRevLowCube);
      addPostoShoulder(ArmPositions.SHELF_PICKUP_ADAPTIVE,  Constants.ArmRotationValues.armRotForShelfCone - 11, Constants.ArmRotationValues.armRotForShelfCube - 11, Constants.ArmRotationValues.armRotRevShelfCone - 11, Constants.ArmRotationValues.armRotRevShelfCube - 11);
      addPostoShoulder(ArmPositions.MID_SCORE_ADAPTIVE ,Constants.ArmRotationValues.armRotForMidCone , 36 ,Constants.ArmRotationValues.armRotRevMidCone ,Constants.ArmRotationValues.armRotRevMidCube );
      addPostoShoulder(ArmPositions.HIGH_SCORE_ADAPTIVE , 0, 47, 133, Constants.ArmRotationValues.armRotRevHighCube);
      addPostoShoulder(ArmPositions.FRAME_PERIMETER ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter ,Constants.ArmRotationValues.framePerimeter );  

      addPostoExtend  (ArmPositions.GROUND_PICKUP_ADAPTIVE,8000, 8000, 8000, 8000);
      addPostoExtend  (ArmPositions.STOWED_ADAPTIVE, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow, Constants.ArmExtendValues.armExtendStow);
      addPostoExtend  (ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.ArmExtendValues.armExtendForLowCone, Constants.ArmExtendValues.armExtendForLowCube, Constants.ArmExtendValues.armExtendRevLowCone, Constants.ArmExtendValues.armExtendRevLowCube);
      addPostoExtend  (ArmPositions.SHELF_PICKUP_ADAPTIVE, Constants.ArmExtendValues.armExtendForShelfCone, Constants.ArmExtendValues.armExtendForShelfCube, Constants.ArmExtendValues.armExtendRevShelfCone, Constants.ArmExtendValues.armExtendRevShelfCube);
      addPostoExtend  (ArmPositions.MID_SCORE_ADAPTIVE, Constants.ArmExtendValues.armExtendForMidCone, Constants.ArmExtendValues.armExtendForMidCube, Constants.ArmExtendValues.armExtendRevMidCone, Constants.ArmExtendValues.armExtendRevMidCube);
      addPostoExtend  (ArmPositions.HIGH_SCORE_ADAPTIVE, 1000, Constants.ArmExtendValues.armExtendForHighCube, Constants.ArmExtendValues.armExtendRevHighCone, Constants.ArmExtendValues.armExtendRevHighCube);
      addPostoExtend  (ArmPositions.FRAME_PERIMETER, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter, Constants.ArmExtendValues.framePerimeter);
      
      addPostoJoint   (ArmPositions.GROUND_PICKUP_ADAPTIVE, 31000, 20000, 31000, 20000);
      addPostoJoint   (ArmPositions.STOWED_ADAPTIVE, Constants.JointRotationValues.JointRotStowCone, Constants.JointRotationValues.JointRotStowCube, Constants.JointRotationValues.JointRotStowCone, Constants.JointRotationValues.JointRotStowCube);
      addPostoJoint   (ArmPositions.GROUND_SCORE_ADAPTIVE, Constants.JointRotationValues.JointRotForLowCone, Constants.JointRotationValues.JointRotForLowCube, Constants.JointRotationValues.JointRotRevLowCone, Constants.JointRotationValues.JointRotRevLowCube);
      addPostoJoint   (ArmPositions.SHELF_PICKUP_ADAPTIVE, Constants.JointRotationValues.JointRotForShelfCone / 0.667, Constants.JointRotationValues.JointRotForShelfCube / 0.667, 42000, Constants.JointRotationValues.JointRotRevShelfCube / 0.667);
      addPostoJoint   (ArmPositions.MID_SCORE_ADAPTIVE, 45000, Constants.JointRotationValues.JointRotForMidCube, 8500, Constants.JointRotationValues.JointRotRevMidCube);
      addPostoJoint   (ArmPositions.HIGH_SCORE_ADAPTIVE, 0, Constants.JointRotationValues.JointRotForHighCube, Constants.JointRotationValues.JointRotRevHighCone, Constants.JointRotationValues.JointRotRevHighCube);
      addPostoJoint   (ArmPositions.FRAME_PERIMETER, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter, Constants.JointRotationValues.framePerimeter);
  
      addPostoShoulder(ArmPositions.UPRIGHT_CONE, 38,38,38,38);
      addPostoExtend(ArmPositions.UPRIGHT_CONE,0,0,0,0);
      addPostoJoint(ArmPositions.UPRIGHT_CONE, 47000,47000,47000,47000);

      addPostoShoulder(ArmPositions.SINGLE_INTAKE, 35,35,35,35);
      addPostoExtend(ArmPositions.SINGLE_INTAKE,0,0,0,0);
      addPostoJoint(ArmPositions.SINGLE_INTAKE, 24000,12000,24000,12000);

      addPostoShoulder(ArmPositions.SHOOT,101,101,101,101);
      addPostoExtend(ArmPositions.SHOOT, 0,0,0,0);
      addPostoJoint(ArmPositions.SHOOT, 9750,0,9750,0);

      SmartDashboard.putData(this);
  }

  public CommandBase armForward() {
    return this.runEnd(()->moveRotArmPercentOutput(0.08), ()->moveRotArmPercentOutput(0.00));
  }

  public void ArmBrakeMode(NeutralMode mode){
    telescopeArm.setNeutralMode(mode);
    rightArm.setNeutralMode(mode);
    leftArm.setNeutralMode(mode);
    middleArm.setNeutralMode(mode);
  }
 
  double lastTime = 0;
  double lastSpeed = 0;

 public void moveRotArmPosition(double degrees){
  /* 
  final int kMeasuredPosHorizontal = 0;  // HORIZONTAL VALUE
  final double kTicksPerDegree = 360 / 360; 
  double currentPos = testCanCoder.getAbsolutePosition();
  double degree = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree; 
  double radians = java.lang.Math.toRadians(degree);
  double cosineScalar = java.lang.Math.cos(radians);
  
  final double maxGravityFFRet = 0.06; //FORCE REQUIRED TO KEEP ARM AT HORIZONTAL WITH ARM RETRACTED, DETERMINE WITH testFF
  final double maxGravityFFExt = 0.066;  // ^ BUT EXTENDED                  //MAX EXTENSION VALUE
  double endFF = maxGravityFFRet + (telescopeArm.getSelectedSensorPosition() / MAX_EXTENSION * (maxGravityFFExt - maxGravityFFRet)) * cosineScalar;

  leftArm.set(ControlMode.PercentOutput, pArmPID.calculate(testCanCoder.getAbsolutePosition(), degrees), DemandType.ArbitraryFeedForward, endFF);
  //leftArm.set(ControlMode.PercentOutput, armPID.calculate(testCanCoder.getAbsolutePosition(), degrees), DemandType.ArbitraryFeedForward, endFF);
  */

  if (hasReset == false) {
    pArmPID.reset(testCanCoder.getAbsolutePosition()); //DO NOT REMOVE!!! check hasReset for reason
    hasReset = true;
  }

  double pidVal = pArmPID.calculate(testCanCoder.getAbsolutePosition(), degrees);
  double acceleration = (pArmPID.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime);
  double setPoint = (pidVal + armFF.calculate(testCanCoder.getAbsolutePosition(), pArmPID.getSetpoint().velocity, acceleration)) / 12;
  leftArm.set(ControlMode.PercentOutput, setPoint);
  lastSpeed = pArmPID.getSetpoint().velocity;
  lastTime = Timer.getFPGATimestamp();

  armRotGoal = degrees;
 }

  public void moveRotArmPercentOutput(double percent){
    leftArm.set(TalonFXControlMode.PercentOutput, percent);
  }

public void moveTelescopeArmPercentOutput(double percent){
  telescopeArm.set(TalonFXControlMode.PercentOutput, percent);
}

public void moveTelescopeArmPosition(double position){
  telescopeArm.set(TalonFXControlMode.MotionMagic, position);
  armExtendGoal = position;
}

public void moveGripperJointPosition(double position){
  gripperJointFalcon.set(TalonFXControlMode.MotionMagic, position);
  wristRotGoal = position;
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

public void homeGripperJointPos(){
  gripperJointFalcon.setSelectedSensorPosition(0);
}

  public double getArmRotGoal(ArmPositions position) {
    return ShoulderPos.get(position)[robotDirection][gamePiece]-11;
  }

  public void armRotPresetPositions(ArmPositions position){
    moveRotArmPosition(getArmRotGoal(position));
  }

  public double getArmExtendGoal(ArmPositions position) {
    return ExtendPos.get(position)[robotDirection][gamePiece];
  }

  public void armExtendPresetPositions(ArmPositions position){
    moveTelescopeArmPosition(getArmExtendGoal(position));
  }

  public double getJointRotGoal(ArmPositions position) {
    return JointPos.get(position)[robotDirection][gamePiece] * multiplier;
  }

  public void jointRotPresetPositions(ArmPositions position){
    moveGripperJointPosition(getJointRotGoal(position));
  }
  
  public double getArmEncoderPosition(){
    return testCanCoder.getAbsolutePosition();
  }

  public double getGripperJointPos(){
    return gripperJointFalcon.getSelectedSensorPosition();
  }

  public CommandBase manualMoveGripper(DoubleSupplier sup) {
    return this.runEnd(()->moveGripperJointPercentOutput(sup.getAsDouble()), ()->moveGripperJointPercentOutput(0));
  }

  public void cancel() {
    this.getCurrentCommand().cancel();
 }

  @Override
  public void periodic() {
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("^Arm Rotation Goal", ()->{return armRotGoal;}, null);
    builder.addDoubleProperty("^Arm Extension Goal", ()-> {return armExtendGoal;}, null);
    builder.addDoubleProperty("^Wrist Rotation Goal", ()-> {return wristRotGoal;}, null);

    builder.addDoubleProperty("^Arm Rotation Position", ()->testCanCoder.getAbsolutePosition(), null);
    builder.addDoubleProperty("^Arm Extension Position", ()->telescopeArm.getSelectedSensorPosition(), null);
    builder.addDoubleProperty("^Wrist Rotation Position", ()->getGripperJointPos(), null);

    builder.addIntegerProperty("Arm Encoder Status", ()->testCanCoder.getMagnetFieldStrength().value, null);
    builder.addDoubleProperty("Test FF", null, (t)->test=t);
    SmartDashboard.putNumber("ArmSub/Test FF", 0);   
  }
} 
