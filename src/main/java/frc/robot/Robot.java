// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix.motorcontrol.NeutralMode;
import java.util.Arrays;
import java.util.List;
import java.io.IOException;
import java.net.NetworkInterface;
import java.util.Enumeration;
import java.util.ArrayList;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PowerDistribution pdh;

  DigitalInput input = new DigitalInput(0);
  DigitalInput inputb = new DigitalInput(1);
  boolean resetFlag = false;
  public static boolean isCOMP = false;

  private Autos autos;
  @Override
  public void robotInit() {
    pdh = new PowerDistribution(1, ModuleType.kRev);
    m_robotContainer = new RobotContainer();

    PortForwarder.add(5800, "photonvision.local", 5800);

    DriverStation.silenceJoystickConnectionWarning(true);

    autos = new Autos(m_robotContainer);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(Math.abs(m_robotContainer.s_Swerve.db_getYaw())>=90){
      m_robotContainer.armSub.robotDirection = true;
  }
  else{
    m_robotContainer.armSub.robotDirection = false;
  }  
  SmartDashboard.putNumber("Timer", DriverStation.getMatchTime());
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.candle.disabledLed();
    pdh.setSwitchableChannel(false);

  }

  @Override
  public void disabledPeriodic() {
    if(input.get()){
      m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Coast);
    }
    else{
      m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Brake);
    }

    if (inputb.get() != resetFlag) {
      resetFlag = inputb.get();
      m_robotContainer.armSub.homeGripperJointPos();
      m_robotContainer.armSub.homeTelescopePosition();
    }
    m_robotContainer.s_Swerve.gyroOffset = -1 * m_robotContainer.s_Swerve.getPose().getRotation().getDegrees();
    autos.setTraj();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autos.get();
    autos.setTraj();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Brake);
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    m_robotContainer.armSub.ArmBrakeMode(NeutralMode.Brake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //AutoScore.autoSelectTab();
    m_robotContainer.s_Swerve.zeroGyro(); 
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private static List<byte[]> getMacAddresses() throws IOException {
		List<byte[]> macAddresses = new ArrayList<>();

		Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
		// connect to network
		NetworkInterface networkInterface;
		while (networkInterfaces.hasMoreElements()) {
			networkInterface = networkInterfaces.nextElement();

			byte[] address = networkInterface.getHardwareAddress();
			if (address == null) {
				continue;
			}

			macAddresses.add(address);
		}
		return macAddresses;
	}

  static {
		List<byte[]> macAddresses;
		try {
			macAddresses = getMacAddresses();
		} catch (IOException e) {
			System.out.println("Mac Address attempt unsuccessful");
			System.out.println(e);
			macAddresses = List.of();
		}

		for (byte[] macAddress : macAddresses) {
			// first check if we are comp
			if (Arrays.compare(Constants.MAC_ADDRESSES.COMP, macAddress) == 0) {
        System.out.println("Comp Bot Connected!");
				isCOMP = true;
				break;
      }else if (Arrays.compare(Constants.MAC_ADDRESSES.SIM, macAddress) == 0){
        System.out.println("Simulation Connected!");
        break;
			} else {
				System.out.print("New Mac Address Discovered! -> ");
        for (int k = 0; k < macAddress.length; k++) {
          System.out.format("0x%02X%s", macAddress[k], (k < macAddress.length - 1) ? ":" : "");
        }
        System.out.println(".");
			}
		}
  }
}
