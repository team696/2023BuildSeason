// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
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
  private final Trigger panelGPSwitch = new JoystickButton(joystickPanel, 17);
  private final Trigger panelRelease = new JoystickButton(operatorPanel, 15);
  private final Trigger panelEmptyRight = new JoystickButton(operatorPanel, 12);
  private final Trigger panelEmptyLeft = new JoystickButton(operatorPanel, 14);
  private final Trigger panelLED1 = new JoystickButton(operatorPanel, 4);
  private final Trigger panelLED2 = new JoystickButton(operatorPanel, 2);
  private final Trigger panelLED3 = new JoystickButton(operatorPanel, 7);
  private final Trigger panelLED4 = new JoystickButton(operatorPanel, 6);
  private final Trigger panelLED5 = new JoystickButton(operatorPanel, 1);

  



  // private final Trigger TopLeftSwitch = new JoystickButton(operatorPanel)
 
  public final Swerve s_Swerve = new Swerve();
  public final ArmSub armSub = new ArmSub();
  public final Gripper gripper = new Gripper();
  public final CANdleSub candle = new CANdleSub();

 public SendableChooser<Command> m_chooser = new SendableChooser<>();
  // private final SequentialCommandGroup leftSideBlue = new LeftSideAuto(s_Swerve, armSub, gripper);
  // private final SequentialCommandGroup rightSideRed = new LeftSideAutoRed(s_Swerve, armSub, gripper);
  // private final SequentialCommandGroup balanceBlue = new BalanceAuto(s_Swerve, armSub, gripper);
  // private final SequentialCommandGroup balanceRed = new BalanceAutoRed(s_Swerve, armSub, gripper);

  /* Subsystems */

 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    operatorPanel.setOutput(2, true );
    operatorPanel.setOutput(1, true );
    operatorPanel.setOutput(5, true );
    operatorPanel.setOutput(6, true );
    operatorPanel.setOutput(10, true );
    operatorPanel.setOutput(13, true );
    operatorPanel.setOutput(16, true );
    operatorPanel.setOutput(19, true );

    operatorPanel.setOutput(3, true );

    operatorPanel.setOutput(4, true );
    operatorPanel.setOutput(7, true );
    operatorPanel.setOutput(8, true );
    operatorPanel.setOutput(9, true );
    operatorPanel.setOutput(11, true );
    operatorPanel.setOutput(12, true );
    operatorPanel.setOutput(14, true );
    operatorPanel.setOutput(15, true );
    operatorPanel.setOutput(17, true );
    operatorPanel.setOutput(18, true );


    // m_chooser.setDefaultOption("Balance Blue", balanceBlue);
    // m_chooser.addOption("Balance Red", balanceRed);
    // m_chooser.addOption("Left Blue", leftSideBlue);
    // m_chooser.addOption("Right Red", rightSideRed);

    SmartDashboard.putData(m_chooser);
    
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).
    withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // armSub.setDefaultCommand(new HoldArmPos(armSub, ArmPositions.STOWED_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //candle.setDefaultCommand(new InstantCommand(() -> candle.setColor(operatorExtraRight.getAsBoolean())));
    // armSub.setDefaultCommand(new MoveJointTemp(armSub, operatorPanel));
    // armSub.setDefaultCommand(new ArmExtendTest(armSub, operatorPanel));
    // armSub.setDefaultCommand(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 5)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      armSub.setDefaultCommand(new AdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    candle.setDefaultCommand(new SetLedCommand(candle));
    configureButtonBindings();

  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // panelRollers.whileTrue(new GripperCommand(gripper, 0.7)); //Push OUT
    // panelShelf.whileTrue(new GripperCommand(gripper, -1)); // Take IN
    // panelStow.onTrue(new InstantCommand(() -> armSub.homeGripperJointPos()));


    panelHigh.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelMid.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLow.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.GROUND_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // panelStow.onTrue(new AutoPlaceGamePiece(armSub, gripper, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelGround.whileTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelShelf.whileTrue(new ShelfIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRelease.whileTrue(new AdaptiveOuttake(gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRollers.whileTrue(new ManualRollers(gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelEmptyRight.whileTrue(new UprightConeIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelEmptyLeft.whileTrue(new SingleSubstationIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLED3.onTrue(new InstantCommand(() -> armSub.homeTelescopePosition()));
    panelLED4.onTrue(new InstantCommand(() -> armSub.homeGripperJointPos()));

    // panelStow.whileTrue(new AutoBalanceStation(s_Swerve, false, true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // panelStow.whileTrue(new FullArmPosition(armSub, 150, 50000, 9000,  true));
    // panelStow.whileFalse(new FullArmPosition(armSub, 0, 0, 0,  false));

    // panelMid.whileTrue(new ArmExtendPositionTest(armSub, 50000));
    // panelMid.whileFalse(new ArmExtendPositionTest(armSub, 0));

    // panelLow.whileTrue(new ArmRotationTest(armSub, 130).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // panelLow.whileFalse(new ArmRotationTest(armSub, 5).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    //  panelGround.whileTrue(new HoldJointPos(armSub, 11000).alongWith(new ArmExtendPositionTest(armSub, 7000)).alongWith(new ArmRotationTest(armSub, 0)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // panelGround.whileFalse(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 2)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));



    // panelLow.onTrue(new InstantCommand(() -> armSub.homeArmRotPos()));

    // panelStow.whileTrue(new HoldJointPos(armSub, 19).alongWith(new ArmExtendPositionTest(armSub, 5000)).alongWith(new ArmRotationTest(armSub, 0)));
    // panelStow.whileFalse(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 5)));


    
   
    // panelStow.whileTrue(new HoldJointPos(armSub, 15).alongWith(new ArmExtendPositionTest(armSub, 7000)).alongWith(new ArmRotationTest(armSub, 0)));
    // panelStow.whileFalse(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 5)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // panelGround.whileFalse(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 5)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // panelStow.whileTrue(new HoldJointPos(armSub, 11000));
    // panelStow.whileFalse(new HoldJointPos(armSub, 0));
    // panelStow.whileTrue(new HoldJointPos(armSub, 30).alongWith(new ArmExtendPositionTest(armSub, 5000)).alongWith(new ArmRotationTest(armSub, 37)));
    // panelStow.whileFalse(new HoldJointPos(armSub, 0).alongWith(new ArmExtendPositionTest(armSub, 0)).alongWith(new ArmRotationTest(armSub, 5)));


    // panelGPSwitch.whileTrue(new ConeVCube(0));
    // panelGPSwitch.whileFalse(new ConeVCube(1));

    // operatorIntakeUp.whileTrue(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorIntakeUp.whileFalse(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorIntakeDown.whileTrue(new HoldArmPos(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // operatorIntakeDown.whileFalse(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorHook.toggleOnTrue(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorSpinup.whileTrue(new LockToGamePiece(s_Swerve, joystickPanel, translationAxis, strafeAxis, true, false, GlobalVariables.gamePiece).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorLeftEmpty.toggleOnTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // operatorHook.whileTrue(new ArmExtendTest(armSub));
    rightJoy.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // leftJoy.whileTrue(new LockToGamePiece(s_Swerve, joystickPanel, translationAxis, strafeAxis, true, true, GlobalVariables.gamePiece).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    leftJoy.whileTrue(new LockToGrid(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, true, true));
    // buttonX.toggleOnTrue(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // buttonY.whileTrue(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // buttonB.whileTrue(new HoldArmPos(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // buttonA.whileTrue(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // xboxRightJoyButton.whileTrue(new HoldArmPos(armSub, ArmPositions.CONE_UPRIGHT).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));



    // leftBumper.toggleOnTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // leftBumper.whileTrue(new HoldArmPos(armSub, ArmPositions.SHELF_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // rightBumper.toggleOnTrue(new ShelfIntake(gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // operatorDeploy.onTrue(new TurnInPlace(s_Swerve, true, false, 0));
    // operatorDeploy.onTrue(new CustomSwerveTrajectory(s_Swerve, 10, 4, 0, 3, 10));
    // operatorRightEmpty.onTrue(new BalanceAuto(s_Swerve));
    // TODO RED SIDE 
    // LeftLittleButton.onTrue(new AutoPlaceRed(s_Swerve, armSub, gripper)) ;
  // TODO BLUE SIDE 
    // LeftLittleButton.onTrue(new AutoPlaceTest2(s_Swerve, armSub, gripper));
    
  }
/*   _                        
  \`*-.                    
   )  _`-.                 
  .  : `. .   < - PET HERE 
  : _   '  \               
  ; *` _.   `*-._          
  `-.-'          `-.       
    ;       `       `.     
    :.       .        \    
    . \  .   :   .-'   .   
    '  `+.;  ;  '      :   
    :  '  |    ;       ;-. 
    ; '   : :`-:     _.`* ;
[bug] .*' /  .*' ; .*`- +'  `*' 
 `*-*   `*-*  `*-*' */
/*  　　　　 　＿__＿
 　　　 　 　／＞　　フ
 　　　 　　| 　_　 _ l  < - PET HERE 
 　 　　 　／` ミ＿xノ
 　　 　 /　　　 　 |
 　　　 /　 ヽ　　 ﾉ
 　 　 │　　|　|　|
 　／￣|　　 |　|　|
 　| (￣ヽ＿_ヽ_)__)
 　二つ */
  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("ThreePiece",  new PathConstraints(3, 3));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("marker1", new PrintCommand("Passed marker 1"));
eventMap.put("goarm", new AdaptiveArmMovement(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE));
eventMap.put("stowarm", new AutoAdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE));
eventMap.put("placehigh", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.HIGH_SCORE_ADAPTIVE));
eventMap.put("switchcone", new ConeVCube(0));
eventMap.put("switchcube", new ConeVCube(1));
eventMap.put("cubepickup", new AutoGroundIntake(armSub, gripper, 1) );
eventMap.put("conepickup", new AutoGroundIntake(armSub, gripper, 0) );
eventMap.put("placemid", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.MID_SCORE_ADAPTIVE));
eventMap.put("holdstow", new AdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE));
eventMap.put("balance", new AutoBalanceStation(s_Swerve, true, false));
eventMap.put("placelow", new AutoPlaceGamePiece(armSub, gripper, ArmPositions.GROUND_SCORE_ADAPTIVE));







// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(pathGroup);

    // An ExampleCommand will run in autonomous
    // return new BalanceAuto(s_Swerve, armSub, gripper);
    // return new BalanceAutoRed(s_Swerve, armSub, gripper);
    // return m_chooser.getSelected();
    return fullAuto;//new ButterAutoTest(s_Swerve);
    // return new LeftSideAuto(s_Swerve, armSub, gripper);
    // return new LeftSideAutoRed(s_Swerve, armSub, gripper);
  }
}
