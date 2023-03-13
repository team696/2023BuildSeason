// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalVariables.ArmPositions;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Gripper.GripperState;

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
  private final Trigger LeftLittleButton = new JoystickButton(driver, 7);
  private final Trigger RightLittleButton = new JoystickButton(driver, 8);
  private final Trigger buttonX = new JoystickButton(driver, 3);
  private final Trigger buttonY = new JoystickButton(driver, 4);
  private final Trigger buttonB = new JoystickButton(driver, 2);
  private final Trigger buttonA = new JoystickButton(driver, 1);
  private final Trigger leftBumper = new JoystickButton(driver, 5);
  private final Trigger rightBumper = new JoystickButton(driver, 6);
  private final Trigger leftJoy = new JoystickButton(joystickPanel, 1);
  private final Trigger rightJoy = new JoystickButton(joystickPanel, 2);
  public final Trigger operatorDeploy = new JoystickButton(operatorPanel, 11);
  private final Trigger operatorShoot = new JoystickButton(operatorPanel, 3);
  private final Trigger operatorLeftEmpty  = new JoystickButton(operatorPanel, 9);
  private final Trigger operatorRightEmpty = new JoystickButton(operatorPanel, 12);
  private final Trigger operatorLatch = new JoystickButton(operatorPanel, 7);
  private final Trigger operatorHook = new JoystickButton(operatorPanel, 8);
  private final Trigger operatorIntakeUp = new JoystickButton(operatorPanel, 13);
  private final Trigger operatorIntakeDown = new JoystickButton(operatorPanel, 14);
  private final Trigger operatorSpinup = new JoystickButton(operatorPanel, 1);
  private final Trigger operatorExtraRight = new JoystickButton(operatorPanel, 15);
  private final Trigger xboxRightJoyButton = new JoystickButton(driver, 10);

 
  public final Swerve s_Swerve = new Swerve();
  public final ArmSub armSub = new ArmSub();
  public final Gripper gripper = new Gripper();
  public final CANdleSub candle = new CANdleSub();

 public SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SequentialCommandGroup leftSideBlue = new LeftSideAuto(s_Swerve, armSub, gripper);
  private final SequentialCommandGroup rightSideRed = new LeftSideAutoRed(s_Swerve, armSub, gripper);
  private final SequentialCommandGroup balanceBlue = new BalanceAuto(s_Swerve, armSub, gripper);
  private final SequentialCommandGroup balanceRed = new BalanceAutoRed(s_Swerve, armSub, gripper);

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

    m_chooser.setDefaultOption("Balance Blue", balanceBlue);
    m_chooser.addOption("Balance Red", balanceRed);
    m_chooser.addOption("Left Blue", leftSideBlue);
    m_chooser.addOption("Right Red", rightSideRed);

    SmartDashboard.putData(m_chooser);
    
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).
    withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    armSub.setDefaultCommand(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    //candle.setDefaultCommand(new InstantCommand(() -> candle.setColor(operatorExtraRight.getAsBoolean())));
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
    operatorExtraRight.whileTrue(new ConeVCube(0));
    operatorExtraRight.whileFalse(new ConeVCube(1));

    operatorIntakeUp.whileTrue(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorIntakeUp.whileFalse(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorIntakeDown.whileTrue(new HoldArmPos(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    operatorIntakeDown.whileFalse(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorLatch.toggleOnTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CONE)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorHook.toggleOnTrue(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorSpinup.whileTrue(new LockToGamePiece(s_Swerve, joystickPanel, translationAxis, strafeAxis, true, false, GlobalVariables.gamePiece).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorLeftEmpty.toggleOnTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    rightJoy.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    leftJoy.whileTrue(new LockToGamePiece(s_Swerve, joystickPanel, translationAxis, strafeAxis, true, true, GlobalVariables.gamePiece).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    buttonX.toggleOnTrue(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    buttonY.whileTrue(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    buttonB.whileTrue(new HoldArmPos(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    buttonA.whileTrue(new HoldArmPos(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    xboxRightJoyButton.whileTrue(new HoldArmPos(armSub, ArmPositions.CONE_UPRIGHT).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));



    // leftBumper.toggleOnTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    leftBumper.whileTrue(new HoldArmPos(armSub, ArmPositions.SHELF_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    rightBumper.toggleOnTrue(new ShelfIntake(gripper).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // operatorDeploy.onTrue(new TurnInPlace(s_Swerve, true, false, 0));
    operatorDeploy.onTrue(new CustomSwerveTrajectory(s_Swerve, 10, 4, 0, 3, 10));
    // operatorRightEmpty.onTrue(new BalanceAuto(s_Swerve));
    // TODO RED SIDE 
    // LeftLittleButton.onTrue(new AutoPlaceRed(s_Swerve, armSub, gripper)) ;
  // TODO BLUE SIDE 
    LeftLittleButton.onTrue(new AutoPlaceTest2(s_Swerve, armSub, gripper));
    
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new BalanceAuto(s_Swerve, armSub, gripper);
    // return new BalanceAutoRed(s_Swerve, armSub, gripper);
    // return m_chooser.getSelected();
    return new ButterAutoTest(s_Swerve);
    // return new LeftSideAuto(s_Swerve, armSub, gripper);
    // return new LeftSideAutoRed(s_Swerve, armSub, gripper);
  }
}
