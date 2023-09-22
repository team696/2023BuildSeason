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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Constants.ArmPositions;

public class AutoScore extends CommandBase {
  HolonomicDriveController cont = new HolonomicDriveController(new PIDController(5, 0, 0) , new PIDController(5, 0, 0), new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Math.PI * 6, Math.PI * 4)));
  Swerve s;
  ArmSub a;
  Gripper g;
  double t;

  TrajectoryConfig config = new TrajectoryConfig(2.6, 1).setEndVelocity(-0.1).setKinematics(Constants.Swerve.swerveKinematics);
  Trajectory traj;

  CommandBase ap;

  double x;
  double y;
  Rotation2d r;
  double of;
  ArmPositions pp = ArmPositions.HIGH_SCORE_ADAPTIVE;

  Trigger hvm = new JoystickButton(new Joystick(2), 2);

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  DoubleTopic dblTopic = inst.getDoubleTopic("/Dashboard/cock");
  DoubleSubscriber sub = dblTopic.subscribe(1);

  double tv = 0;

  double[] ppp = {5,4.4,3.8,3.25,2.75,2.15,1.6,1.05,0.5};

  public AutoScore(Swerve swerve, ArmSub armSub, Gripper gripper) {
    s = swerve;
    a = armSub;
    g = gripper;
    addRequirements(swerve);

    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch ((int)Math.floor(getSelected() / 9)) {
      case 0:
        pp = ArmPositions.GROUND_SCORE_ADAPTIVE; //LOW
        break; 
      case 1:
        pp = ArmPositions.MID_SCORE_ADAPTIVE; //MID
        break;
      case 2: 
        pp = ArmPositions.HIGH_SCORE_ADAPTIVE; //HIGH
        break;
      default:
        throw new IllegalArgumentException(); // HOW TF DID WE GET THAT
    }
    t = Timer.getFPGATimestamp();       
    Pose2d curPose = s.getPose();
    Alliance al = DriverStation.getAlliance();
    of = (0.3/2 - 0.055 - g.getDistanceSensorM()) * (al == DriverStation.Alliance.Red ? -1 : 1);
    x = (al == DriverStation.Alliance.Red ? 14.82 : 1.5); 
    y = ppp[al==DriverStation.Alliance.Red ? 8-(int)((getSelected())%9) : (int)((getSelected())%9)];
    r = new Rotation2d(al == DriverStation.Alliance.Red ? 0 : Math.PI);
    Rotation2d nr = new Rotation2d(Math.abs(curPose.getX() - x) < 2 ? ((y > curPose.getY() ? Math.PI/2 : Math.PI/2 * 3)) : (r.getRadians()));
    Pose2d newPose = new Pose2d(curPose.getTranslation(), nr);

    traj = TrajectoryGenerator.generateTrajectory(List.of(newPose, new Pose2d(x, y, r)), config);
    s.m_fieldSim.getObject("traj").setTrajectory(traj);
    ap = new AutoPlace(a, g, pp);

    a.resetPID();
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
    builder.addDoubleProperty("Score Position", this::getSelected, null);
    builder.addIntegerProperty("Score Position Setter", null, (t)->tv = t);
    SmartDashboard.putNumber("AutoScore/Score Position Setter", 1);
  }

  private double getSelected() {
    return tv;//sub.get();
  }
}
