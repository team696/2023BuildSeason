// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
@SuppressWarnings("unused")
public class RobotContainer {

  public final Swerve s_Swerve = new Swerve();
  public final ArmSub armSub = new ArmSub();
  public final Gripper gripper = new Gripper();
  public final CANdleSub candle = new CANdleSub();

  /* Controllers */
  private final Joystick driver = new Joystick(1);
  private final Joystick joystickPanel = new Joystick(0);
  public  final Joystick operatorPanel = new Joystick(2);
 
  /* Drive Controls */
  private final int translationAxis =  1;
  private final int strafeAxis =  0;
  private final int rotationAxis =  2;

  /* Driver Buttons */
  private final Trigger leftJoy = new JoystickButton(joystickPanel, 1);
  private final Trigger rightJoy = new JoystickButton(joystickPanel, 2);
  public final Trigger operatorDeploy = new JoystickButton(operatorPanel, 11);
  private final Trigger panelRollers = new JoystickButton(operatorPanel, 10);
  private final Trigger panelShelf = new JoystickButton(operatorPanel, 9);
  private final Trigger panelHigh = new JoystickButton(operatorPanel, 13);
  private final Trigger panelMid = new JoystickButton(operatorPanel, 11);
  private final Trigger panelLow = new JoystickButton(operatorPanel, 3);  
  private final Trigger panelStow = new JoystickButton(operatorPanel, 5);
  private final Trigger panelGround = new JoystickButton(operatorPanel, 16);
  private final Trigger panelGPSwitch = new JoystickButton(operatorPanel, 17);
  private final Trigger panelRelease = new JoystickButton(operatorPanel, 15);
  private final Trigger panelEmptyRight = new JoystickButton(operatorPanel, 12);
  private final Trigger panelEmptyLeft = new JoystickButton(operatorPanel, 14);
  private final Trigger panelLED1 = new JoystickButton(operatorPanel, 4);
  private final Trigger panelLED2 = new JoystickButton(operatorPanel, 2); // don't use
  private final Trigger panelLED3 = new JoystickButton(operatorPanel, 7);
  private final Trigger panelLED4 = new JoystickButton(operatorPanel, 6);
  private final Trigger panelLED5 = new JoystickButton(operatorPanel, 1);
  private final Trigger panelLock = new JoystickButton(operatorPanel, 8);

 public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    CONFIGUREAUTOSHIT();
    
    armSub.setDefaultCommand(new AdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    candle.setDefaultCommand(candle.setColorC(GlobalVariables.gamePiece == 0));
    gripper.setDefaultCommand(new ConstantHold(gripper));
    configureButtonBindings();

    SmartDashboard.putData("AUTOS", m_chooser);
  }
  
  private void configureButtonBindings() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, true, true).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    panelHigh.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelMid.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLow.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.GROUND_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelStow.onTrue(new AutoPlaceGamePiece(armSub, gripper, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelGround.whileTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelShelf.whileTrue(new ShelfIntake(armSub).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRelease.whileTrue(new AdaptiveOuttake(gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRollers.whileTrue(new ManualRollers(gripper, candle).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelEmptyRight.whileTrue(new UprightConeIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelEmptyLeft.whileTrue(new SingleSubstationIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLED3.onTrue(new InstantCommand(() -> armSub.homeTelescopePosition()));
    panelLED4.onTrue(new InstantCommand(() -> armSub.homeGripperJointPos()));
    panelLock.whileTrue(new MoveJointTemp(armSub, operatorPanel).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLED5.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.FRAME_PERIMETER).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelGPSwitch.onTrue(new InstantCommand(()-> GlobalVariables.gamePiece = 0));
    panelGPSwitch.onFalse(new InstantCommand(()-> GlobalVariables.gamePiece = 1));
    rightJoy.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));   
    panelLED1.onTrue(new AutoScore(s_Swerve, armSub, gripper).andThen(new AutoPlaceGamePiece(armSub, gripper, ArmPositions.MID_SCORE_CONE)));

    operatorPanel.setOutputs(Integer.MAX_VALUE);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private void CONFIGUREAUTOSHIT() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("goarm", new AdaptiveArmMovement(armSub, ArmPositions.MID_SCORE_ADAPTIVE));
    eventMap.put("stowarm", new AutoAdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE));
    eventMap.put("placehigh", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.HIGH_SCORE_ADAPTIVE));
    eventMap.put("switchcone", new InstantCommand(() -> GlobalVariables.gamePiece = 0));
    eventMap.put("switchcube", new InstantCommand(() -> GlobalVariables.gamePiece = 1));
    eventMap.put("cubepickup", new AutoGroundIntake(armSub, gripper, 1) );
    eventMap.put("conepickup", new AutoGroundIntake(armSub, gripper, 0) );
    eventMap.put("placemid", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.MID_SCORE_ADAPTIVE));
    eventMap.put("holdstow", new AdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE));
    eventMap.put("balance", new AutoBalanceStation(s_Swerve));
    eventMap.put("placelow", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.GROUND_SCORE_ADAPTIVE));
    eventMap.put("shootcone", new AutoShootCone(armSub, gripper));
  
    final SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
  
    final List<PathPlannerTrajectory> CableProtector = PathPlanner.loadPathGroup("CableProtector", new PathConstraints(3  , 3), new PathConstraints(0.5, 1),new PathConstraints(3  , 3));
    final List<PathPlannerTrajectory> ThreePiece = PathPlanner.loadPathGroup("ThreePieceCone",  new PathConstraints(3, 3));
    final List<PathPlannerTrajectory> TwoPieceClimb = PathPlanner.loadPathGroup("FullAuto",  new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3), new PathConstraints(2, 2));
    final List<PathPlannerTrajectory> MiddleOnePiece = PathPlanner.loadPathGroup("MiddleOnePiece",  new PathConstraints(2, 2));
    final List<PathPlannerTrajectory> MiddleTwoPiece = PathPlanner.loadPathGroup("MiddleTwoPiece",  new PathConstraints(2, 2));
    final List<PathPlannerTrajectory> CableProtectorClimb = PathPlanner.loadPathGroup("CableProtectorCharge", new PathConstraints(3  , 3), new PathConstraints(0.5, 1),new PathConstraints(3  , 3));
    final List<PathPlannerTrajectory> CableProtectorDoubleHigh = PathPlanner.loadPathGroup("CableProtectorDoubleHigh", new PathConstraints(3.5  , 3), new PathConstraints(0.5, 1),new PathConstraints(3.5  , 3),new PathConstraints(3.3  , 3.3),new PathConstraints(0.5, 1),new PathConstraints(3.0  , 3.0));
  
    final Command cCableProtector = autoBuilder.fullAuto(CableProtector);
    final Command cThreePiece = autoBuilder.fullAuto(ThreePiece);
    final Command cTwoPieceClimb = autoBuilder.fullAuto(TwoPieceClimb);
    final Command cMiddleOnePiece = autoBuilder.fullAuto(MiddleOnePiece).andThen(new AutoBalanceStation(s_Swerve));
    final Command cMiddleTwoPiece = autoBuilder.fullAuto(MiddleTwoPiece).andThen(new AutoBalanceStation(s_Swerve));
    final Command cCableProtectorClimb = autoBuilder.fullAuto(CableProtectorClimb);
    final Command cCableProtectorDoubleHigh = autoBuilder.fullAuto(CableProtectorDoubleHigh);
  
    m_chooser.setDefaultOption("Cable Protector Shoot 3", cCableProtector);
    m_chooser.addOption("Three Piece", cThreePiece);
    m_chooser.addOption("Two Piece Climb", cTwoPieceClimb);
    m_chooser.addOption("Middle One Piece", cMiddleOnePiece);
    m_chooser.addOption("Middle Two Piece", cMiddleTwoPiece);
    m_chooser.addOption("Cable Protector Climb", cCableProtectorClimb);
    m_chooser.addOption("Cable Protector Double High", cCableProtectorDoubleHigh);
  }
}






/*
 *  TODO:
 *      POTENTIALLY ADD AUTO LINK SCORING DETECTION -> PRIORITIZE CERTAIN POSITIONS OVER OTHERS
 *      FUCK WITH ALL JOINT PIDS AND MESS WITH ARBITRARY FEED FORWARD
 *      CLEAN CODE !!!!
 * 
 */