// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.math.Conversions;
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

  private PowerDistribution pdh;
  DigitalInput input = new DigitalInput(0);



  // PhotonCamera camera = new PhotonCamera("photonvision");

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    candlesub = new CANdleSub();
    pdh = new PowerDistribution(1, ModuleType.kRev);
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

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if(input.get()){
      m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Coast);
    }
    else{
      m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Brake);

    }
    candlesub.disabledLed();
    pdh.setSwitchableChannel(false);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.s_Swerve.resetOdometry(new Pose2d(Units.inchesToMeters(72), Units.inchesToMeters(176), new Rotation2d(0)));

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Brake);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_robotContainer.s_Swerve.updateOdometry();

  }

  @Override
  public void teleopInit() {
    // m_robotContainer.s_Swerve.normalizeOdometry();
    // m_robotContainer.s_Swerve.autoZeroGyro();
    m_robotContainer.s_Swerve.zeroGyro();
    m_robotContainer.s_Swerve.resetOdometry(new Pose2d(Units.inchesToMeters(72), Units.inchesToMeters(176), new Rotation2d(0)));
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
    Alliance alliance;
alliance = DriverStation.getAlliance();
m_robotContainer.s_Swerve.updateOdometry();
pdh.setSwitchableChannel(true);
// System.out.println(alliance);

    // candlesub.enabledLed();


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
