// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class AutoScore extends CommandBase {
  /** Creates a new AutoScore. */
  HolonomicDriveController cont = new HolonomicDriveController(new PIDController(5, 0, 0) , new PIDController(5, 0, 0), new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 6, Math.PI * 4)));
  Swerve s;
  ArmSub a;
  Gripper g;
  double t;

  TrajectoryConfig config = new TrajectoryConfig(2.6, 1).setEndVelocity(0).setKinematics(Constants.Swerve.swerveKinematics);
  Trajectory traj;

  CommandBase ap;

  double x;
  double y;
  Rotation2d r;
  double of;
  ArmPositions pp = ArmPositions.HIGH_SCORE_ADAPTIVE;

  Trigger hvm = new JoystickButton(new Joystick(2), 2);

  ShuffleboardTab redt = Shuffleboard.getTab("RED");
  ShuffleboardTab bluet = Shuffleboard.getTab("BLUE");

  public class pair {
    public double y = 0;
    public boolean whiteList = true;
    public boolean whiteListM = true;

    public GenericEntry buttonH;
    public GenericEntry buttonM;

    public GenericEntry buttonHB;
    public GenericEntry buttonMB;

    public pair(double y_) {
      y = y_;
    }
  }
        //CUBO or CONO, POSITION, 
  pair p[][] = { 
      { 
        new pair(5), new pair(3.8), new pair(3.25), new pair(2.15), new pair(1.6), new pair(0.5) 
      },
      {
        new pair(4.4), new pair(2.75), new pair(1.05)
      }, 
  };

  double nearestPoint (pair p[], double val) {
    double minDist = Double.MAX_VALUE;
    double bestPoint = -1;
    for (pair t : p) {
      if (((pp == ArmPositions.HIGH_SCORE_ADAPTIVE && t.whiteList) || (pp == ArmPositions.MID_SCORE_ADAPTIVE && t.whiteListM)) && Math.abs(t.y - val) < minDist) {
        minDist = t.y - val;
        bestPoint = t.y;
      }
    }
    return bestPoint;
  }

  public AutoScore(Swerve swerve, ArmSub armSub, Gripper gripper) {
    s = swerve;
    a = armSub;
    g = gripper;
    addRequirements(swerve);

    SmartDashboard.putData(this);

    Shuffleboard.selectTab("RED");
  }

  int o[][] = { {0,0}, {1,0}, {0,1}, {0,2}, {1,1}, {0,3}, {0,4}, {1,2}, {0,5}};

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pp = (hvm.getAsBoolean()?ArmPositions.HIGH_SCORE_ADAPTIVE : ArmPositions.MID_SCORE_ADAPTIVE);

    t = Timer.getFPGATimestamp();       
    Pose2d curPose = s.getPose();
    Alliance al = DriverStation.getAlliance();

    for (int j = 0; j < 9; j++) {
      if (al == Alliance.Red) {
        p[o[j][0]][o[j][1]].whiteList = p[o[j][0]][o[j][1]].buttonH.getBoolean(true);
        p[o[j][0]][o[j][1]].whiteListM = p[o[j][0]][o[j][1]].buttonM.getBoolean(true);
      }else {
        p[o[j][0]][o[j][1]].whiteList = p[o[j][0]][o[j][1]].buttonHB.getBoolean(true);
        p[o[j][0]][o[j][1]].whiteListM = p[o[j][0]][o[j][1]].buttonMB.getBoolean(true);
      }
    }
    of = (0.3
    /2 - 0.055 - g.getDistanceSensorM()) * (al == DriverStation.Alliance.Red ? -1 : 1);
    x = (al == DriverStation.Alliance.Red ? 14.82 : 1.5); 
    y = nearestPoint(p[GlobalVariables.gamePiece], curPose.getY()) + of;
    r = new Rotation2d(al == DriverStation.Alliance.Red ? 0 : Math.PI);
    //whole dist / 2 + radius of cone + sensor dist
    for (pair[] j : p) 
      for(pair k : j)
        s.m_fieldSim.getObject(String.valueOf(k.y)).setPose(0,0,new Rotation2d(0));

    if (y == -1) return;

    Rotation2d nr = new Rotation2d(Math.abs(curPose.getX() - x) < 2 ? ((y > curPose.getY() ? Math.PI/2 : Math.PI/2 * 3)) : (r.getRadians()));
    Pose2d newPose = new Pose2d(curPose.getTranslation(), nr);

    traj = TrajectoryGenerator.generateTrajectory(List.of(newPose, new Pose2d(x, y, r)), config);
    s.m_fieldSim.getObject("traj").setTrajectory(traj);

    for (pair m : p[GlobalVariables.gamePiece]) { 
      if (((pp == ArmPositions.HIGH_SCORE_ADAPTIVE && m.whiteList == false) || (pp == ArmPositions.MID_SCORE_ADAPTIVE && m.whiteListM == false))) continue;
      s.m_fieldSim.getObject(String.valueOf(m.y)).setPose(x, m.y, r);
    }

    ap = new AutoPlaceGamePiece(a, g, pp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (traj == null) return;

    Trajectory.State goal = traj.sample(Timer.getFPGATimestamp() - t); 
    
    ChassisSpeeds cs = cont.calculate(s.getPose(), goal, new Rotation2d(Math.PI + r.getRadians()));

    s.Drive(cs, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted != true && traj != null)
      ap.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (traj == null || Timer.getFPGATimestamp() - t > traj.getTotalTimeSeconds());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    for (int j = 0; j < 9; j++) {
      pair k = p[o[j][0]][o[j][1]];
      //builder.addBooleanProperty("H" + Double.toString(k.y), () -> {return k.whiteList;}, (b) -> {k.whiteList = b;});
      //builder.addBooleanProperty("M" + Double.toString(k.y), () -> {return k.whiteListM;}, (b) -> {k.whiteListM = b;});         

      p[o[j][0]][o[j][1]].buttonH = redt.add("H" + Double.toString(k.y), true).withPosition(8-j, 0).withWidget("Toggle Button").getEntry();
      p[o[j][0]][o[j][1]].buttonM = redt.add("M" + Double.toString(k.y), true).withPosition(8-j, 1).withWidget("Toggle Button").getEntry();

      p[o[j][0]][o[j][1]].buttonHB = bluet.add("H" + Double.toString(k.y), true).withPosition(j, 0).withWidget("Toggle Button").getEntry();
      p[o[j][0]][o[j][1]].buttonMB = bluet.add("M" + Double.toString(k.y), true).withPosition(j, 1).withWidget("Toggle Button").getEntry();
    }
  }

  public static void autoSelectTab() {
    if (DriverStation.getAlliance() == Alliance.Red){
      Shuffleboard.selectTab("RED");
    } else {
      Shuffleboard.selectTab("BLUE");
    }
  }
}
