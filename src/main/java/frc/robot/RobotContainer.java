// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  /* Drive Controls */
  private final int translationAxis = /* XboxController.Axis.kLeftY.value */ 1;
  private final int strafeAxis = /* XboxController.Axis.kLeftX.value */ 0;
  private final int rotationAxis = /* XboxController.Axis.kRightX.value */ 4;

  /* Driver Buttons */
  private final Trigger zeroGyro = new JoystickButton(driver, 6);
  private final Trigger buttonX = new JoystickButton(driver, 3);
  private final Trigger buttonY = new JoystickButton(driver, 4);
  private final Trigger buttonB = new JoystickButton(driver, 2);
  private final Trigger buttonA = new JoystickButton(driver, 1);
  private final Trigger gripperRev = new JoystickButton(driver, 5);
  private final Trigger gripperFor = new JoystickButton(driver, 7);




  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final ArmSub armSub = new ArmSub();
  public final Gripper gripper = new Gripper();
 


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    armSub.setDefaultCommand(new ArmPercentageCommand(armSub, 3, 2, driver));
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
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    // gripperFor.whileTrue(new exampleAuto(s_Swerve));
    buttonA.whileTrue(new ArmPositionCommand(armSub, ArmPositions.STOWED));
    buttonB.whileTrue(new ArmPositionCommand(armSub, ArmPositions.MID_SCORE));
    // buttonB.whileTrue(new ArmPositionCommand(armSub, ArmPositions.HIGH_SCORE));
    // buttonA.whileTrue(new ArmPositionCommand(armSub, ArmPositions.SHELF_PICKUP));

    buttonX.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.OPEN)));
    buttonY.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CONE)));
    // buttonB.onTrue(new InstantCommand(() -> gripper.setClaw(GripperState.CUBE)));

    gripperFor.whileTrue(new GripperCommand(gripper, 0.8));
   

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
