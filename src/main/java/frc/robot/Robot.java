// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.exampleAuto2;
import frc.robot.subsystems.CANdleSub;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANdleSub candlesub;


  // PhotonCamera camera = new PhotonCamera("photonvision");

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    candlesub = new CANdleSub();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
     m_robotContainer.s_Swerve.updateOdometry();

    //  m_robotContainer.operatorPanel.setOutput(2, true );
    //  m_robotContainer.operatorPanel.setOutput(1, true );
    //  m_robotContainer.operatorPanel.setOutput(5, true );
    //  m_robotContainer.operatorPanel.setOutput(6, true );
    //  m_robotContainer.operatorPanel.setOutput(10, true );
    //  m_robotContainer.operatorPanel.setOutput(13, true );
    //  m_robotContainer.operatorPanel.setOutput(16, true );
 
 
    //  m_robotContainer.operatorPanel.setOutput(3, true );
    //  m_robotContainer.operatorPanel.setOutput(4, true );
    //  m_robotContainer.operatorPanel.setOutput(7, true );
    //  m_robotContainer.operatorPanel.setOutput(8, true );
    //  m_robotContainer.operatorPanel.setOutput(9, true );
    //  m_robotContainer.operatorPanel.setOutput(11, true );
    //  m_robotContainer.operatorPanel.setOutput(12, true );
    //  m_robotContainer.operatorPanel.setOutput(14, true );
    //  m_robotContainer.operatorPanel.setOutput(15, true );
    //  m_robotContainer.operatorPanel.setOutput(17, true );
    //  m_robotContainer.operatorPanel.setOutput(18, true );
    //  m_robotContainer.operatorPanel.setOutput(19, true );
    //  m_robotContainer.operatorPanel.setOutput(20, true );
    //  m_robotContainer.operatorPanel.setOutput(21, true );
    //  m_robotContainer.operatorPanel.setOutput(22, true );
    //  m_robotContainer.operatorPanel.setOutput(23, true );
    //  m_robotContainer.operatorPanel.setOutput(24, true );
    //  m_robotContainer.operatorPanel.setOutput(25, true );
    //  m_robotContainer.operatorPanel.setOutput(26, true );
    //  m_robotContainer.operatorPanel.setOutput(27, true );
    //  m_robotContainer.operatorPanel.setOutput(28, true );
    //  m_robotContainer.operatorPanel.setOutput(29, true );
    //  m_robotContainer.operatorPanel.setOutput(30, true );
    //  m_robotContainer.operatorPanel.setOutput(31, true );
    //  m_robotContainer.operatorPanel.setOutput(32, true );
    
    CommandScheduler.getInstance().run();
    
   // m_robotContainer.
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    candlesub.disabledLed();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    candlesub.enabledLed();


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
