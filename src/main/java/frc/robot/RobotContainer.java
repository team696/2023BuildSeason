// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
  private final int translationAxis = /* XboxController.Axis.kLeftY.value */ 1;
  private final int strafeAxis = /* XboxController.Axis.kLeftX.value */ 0;
  private final int rotationAxis = /* XboxController.Axis.kRightX.value */ 2;

  /* Driver Buttons */
  private final Trigger zeroGyro = new JoystickButton(driver, 6);
  private final Trigger buttonX = new JoystickButton(driver, 3);
  private final Trigger buttonY = new JoystickButton(driver, 4);
  private final Trigger buttonB = new JoystickButton(driver, 2);
  private final Trigger buttonA = new JoystickButton(driver, 1);
  private final Trigger gripperRev = new JoystickButton(driver, 5);
  private final Trigger gripperFor = new JoystickButton(driver, 7);
  private final Trigger leftJoy = new JoystickButton(joystickPanel, 1);
  private final Trigger rightJoy = new JoystickButton(joystickPanel, 2);
  public final Trigger operatorDeploy = new JoystickButton(operatorPanel, 11);
  private final Trigger operatorShoot = new JoystickButton(operatorPanel, 3);
  private final Trigger operatorLeftEmpty  = new JoystickButton(operatorPanel, 9);
  private final Trigger operatorRightEmpty = new JoystickButton(operatorPanel, 11);
  private final Trigger operatorLatch = new JoystickButton(operatorPanel, 7);
  private final Trigger operatorHook = new JoystickButton(operatorPanel, 8);
  private final Trigger operatorIntakeUp = new JoystickButton(operatorPanel, 13);
  private final Trigger operatorIntakeDown = new JoystickButton(operatorPanel, 14);


  // XboxController test = new XboxController(0);
  
  private SequentialCommandGroup placeCommand;
  




  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final ArmSub armSub = new ArmSub();
  public final Gripper gripper = new Gripper();
  public final CANdleSub candle = new CANdleSub();
 


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


    






    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).
    withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // armSub.setDefaultCommand(new ArmPositionCommand(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // armSub.setDefaultCommand(new ArmPercentageCommand(armSub, 3, 2, driver));

    placeCommand = new AutoPlaceTest2(s_Swerve, armSub, gripper);
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    leftJoy.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // buttonA.toggleOnTrue(new ArmPositionCommand(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // buttonB.toggleOnTrue(new ArmPositionCommand(armSub, ArmPositions.MID_SCORE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    buttonX.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    buttonY.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CONE)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    gripperRev.whileTrue(new GripperCommand(gripper, 0.8));

    gripperFor.toggleOnTrue(new GroundIntake(armSub, gripper).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // operatorIntakeUp.whileTrue(new HoldArmPos(armSub, ArmPositions.STOWED).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorIntakeUp.whileFalse(new HoldArmPos(armSub, ArmPositions.GROUND_PICKUP).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    // operatorIntakeDown.whileTrue(new HoldArmPos(armSub, ArmPositions.MID_SCORE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    operatorLatch.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CONE)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorHook.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CUBE)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    operatorShoot.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // operatorLatch.ont(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));



    // rightJoy.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(1.82942, 0.21511), new Rotation2d(Math.toRadians(180))))));
//.andThen(new ArmPositionCommand(armSub, ArmPositions.MID_SCORE)

    
    
    // operatorDeploy.onTrue(new Test(s_Swerve).andThen(new InstantCommand(() -> gripper.setClaw(GripperState.CONE))));
    // operatorDeploy.onTrue(new ArmPositionCommand(armSub, ArmPositions.MID_SCORE));
    // operatorDeploy.onTrue(new WaitCommand(2).andThen(new InstantCommand(() -> gripper.setClaw(GripperState.CONE))));
    operatorDeploy.onTrue(placeCommand);
    // operatorDeploy.onTrue(new exampleAuto(s_Swerve, rotationAxis, strafeAxis, rotationAxis));
    
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
//  　　　　 　　 ＿__＿
//  　　　 　 　／＞　　フ
//  　　　 　　| 　_　 _ l  < - PET HERE 
//  　 　　 　／` ミ＿xノ
//  　　 　 /　　　 　 |
//  　　　 /　 ヽ　　 ﾉ
//  　 　 │　　|　|　|
//  　／￣|　　 |　|　|
//  　| (￣ヽ＿_ヽ_)__)
//  　二つ
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto2(s_Swerve);
  }
}
