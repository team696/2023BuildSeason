// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.Constants.ArmPositions;
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
  public final Trigger panelGPSwitch = new JoystickButton(operatorPanel, 17);
  private final Trigger panelRelease = new JoystickButton(operatorPanel, 15);
  private final Trigger panelEmptyRight = new JoystickButton(operatorPanel, 12);
  private final Trigger panelEmptyLeft = new JoystickButton(operatorPanel, 14);
  private final Trigger panelLED1 = new JoystickButton(operatorPanel, 4);
  private final Trigger panelLED2 = new JoystickButton(operatorPanel, 2); // don't use
  private final Trigger panelLED3 = new JoystickButton(operatorPanel, 7);
  private final Trigger panelLED4 = new JoystickButton(operatorPanel, 6);
  private final Trigger panelLED5 = new JoystickButton(operatorPanel, 1);
  private final Trigger panelLock = new JoystickButton(operatorPanel, 8);

  public RobotContainer() {    
    armSub.setDefaultCommand(new AdaptiveArmMovement(armSub, ArmPositions.STOWED_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, joystickPanel, translationAxis, strafeAxis, rotationAxis, true, true).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    panelHigh.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.HIGH_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelMid.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.MID_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelLow.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.GROUND_SCORE_ADAPTIVE).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelGround.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.GROUND_PICKUP_ADAPTIVE).alongWith(gripper.intake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); 
    panelShelf.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.SHELF_PICKUP_ADAPTIVE).alongWith(gripper.intake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRelease.whileTrue(gripper.spit().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelRollers.whileTrue(gripper.intake().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    panelEmptyRight.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.UPRIGHT_CONE).alongWith(gripper.intake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); 
    panelEmptyLeft.whileTrue(new AdaptiveArmMovement(armSub, ArmPositions.SINGLE_INTAKE).alongWith(gripper.intake()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); 

    panelLED3.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));    
    //panelLED3.onTrue(new InstantCommand(() -> armSub.homeTelescopePosition()));
    // panelLED4.onTrue(new InstantCommand(() -> armSub.homeGripperJointPos()));
    panelLock.whileTrue(armSub.manualMoveGripper(()->operatorPanel.getRawAxis(0)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)); 
      
    rightJoy.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));    
    panelLED1.onTrue(new AutoScore(s_Swerve, armSub, gripper));

    operatorPanel.setOutputs(Integer.MAX_VALUE);
  }
}




/* 
 *       STUFF TO TEST
 *      
 *      TEST AUTO WITHOUT RESETODOMETRY FUNCTION
 *      TEST AUTO ZERO GYRO BOTH TEAMS
 *      TEST ALL FEATURES
 *      TEST AUTO SCORE + DASHBOARD
 */