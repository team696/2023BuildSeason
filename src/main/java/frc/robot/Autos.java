package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.AdaptiveArmMovement;
import frc.robot.commands.AutoAdaptiveArmMovement;
import frc.robot.commands.AutoBalanceStation;
import frc.robot.commands.AutoGroundIntake;
import frc.robot.commands.AutoPlaceGamePiece;
import frc.robot.commands.AutoShootCone;

public class Autos {
    private List<autoshit> autos = new ArrayList<autoshit>();
    private HashMap<String, autoshit> autoMap = new HashMap<String, autoshit>();
    private RobotContainer container;

    private SwerveAutoBuilder autoBuilder;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StringTopic dblTopic = inst.getStringTopic("/Dashboard/auto selected");
    StringSubscriber sub = dblTopic.subscribe("none");  

    class autoshit {
      public String name = "";
      public List<PathPlannerTrajectory> traj = new ArrayList<PathPlannerTrajectory>();
      public Command command = new WaitCommand(1);

      public autoshit(String name) {
        this.name = name;
      }
      public autoshit build(String hname, PathConstraints constraint, PathConstraints... constraints) {
        this.traj = PathPlanner.loadPathGroup(hname, false, constraint, constraints);
        this.command = autoBuilder.fullAuto(this.traj);
        return this;
      } 
      public autoshit addEndCommand(Command end) {
        this.command.andThen(end);
        return this;
      }
      public void end() {
        autos.add(this);
      }
    }

    public Command get() {
        return autoMap.get(sub.get()).command;
    }

    public void setTraj() {
        Trajectory p = new PathPlannerTrajectory();
        if (sub.get() != null) {
            for (int i = 0; i < autoMap.get(sub.get()).traj.size(); ++i) {
                p = p.concatenate(autoMap.get(sub.get()).traj.get(i));
            }
        }
        container.s_Swerve.m_fieldSim.getObject("traj").setTrajectory(p);
    }

    public Autos(RobotContainer container){
        this.container = container;
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("goarm", new AdaptiveArmMovement(container.armSub, ArmPositions.MID_SCORE_ADAPTIVE));
        eventMap.put("stowarm", new AutoAdaptiveArmMovement(container.armSub, ArmPositions.STOWED_ADAPTIVE));
        eventMap.put("placehigh", new AutoPlaceGamePiece(container.armSub, container.gripper, ArmPositions.HIGH_SCORE_ADAPTIVE));
        eventMap.put("switchcone", new InstantCommand(() -> GlobalVariables.gamePiece = 0));
        eventMap.put("switchcube", new InstantCommand(() -> GlobalVariables.gamePiece = 1));
        eventMap.put("cubepickup", new AutoGroundIntake(container.armSub, container.gripper, 1) );
        eventMap.put("conepickup", new AutoGroundIntake(container.armSub, container.gripper, 0) );
        eventMap.put("placemid", new AutoPlaceGamePiece(container.armSub, container.gripper, ArmPositions.MID_SCORE_ADAPTIVE));
        eventMap.put("holdstow", new AdaptiveArmMovement(container.armSub, ArmPositions.STOWED_ADAPTIVE));
        eventMap.put("balance", new AutoBalanceStation(container.s_Swerve));
        eventMap.put("placelow", new AutoPlaceGamePiece(container.armSub, container.gripper, ArmPositions.GROUND_SCORE_ADAPTIVE));
        eventMap.put("shootcone", new AutoShootCone(container.armSub, container.gripper));

        autoBuilder = new SwerveAutoBuilder(
            container.s_Swerve::getPose, // Pose2d supplier
            container.s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(1.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            container.s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            container.s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );
        
        new autoshit("none").end();
        new autoshit("Cable Protector Shoot 3").build("CableProtector", new PathConstraints(3, 3), new PathConstraints(0.5, 1),new PathConstraints(3, 3)).end();
        new autoshit("Three Piece").build("ThreePieceCone",new PathConstraints(3, 3)).end();
        new autoshit("Two Piece Climb").build("FullAuto",new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3), new PathConstraints(2, 2)).end();
        new autoshit("Middle One Piece").build("MiddleOnePiece",new PathConstraints(2, 2)).end();
        new autoshit("Middle Two Piece").build("MiddleTwoPiece",new PathConstraints(2, 2)).end();
        new autoshit("Cable Protector Climb").build("CableProtectorCharge",new PathConstraints(3  , 3), new PathConstraints(0.5, 1),new PathConstraints(3  , 3)).end();
        new autoshit("Cable Protector Double High").build("CableProtectorDoubleHigh",new PathConstraints(3.5  , 3), new PathConstraints(0.5, 1),new PathConstraints(3.5  , 3),new PathConstraints(3.3  , 3.3),new PathConstraints(0.5, 1),new PathConstraints(3.0  , 3.0)).end();
        
        String[] names = new String[autos.size()];

        for (int i = 0; i < autos.size(); i++) {
            autoMap.put(autos.get(i).name, autos.get(i));
            names[i] = autos.get(i).name;
        }
        
        SmartDashboard.putStringArray("Auto List", names);
    }
}
