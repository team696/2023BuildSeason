package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AdaptiveArmMovement;
import frc.robot.commands.AutoBalanceStation;
import frc.robot.commands.AutoPlace;
import frc.robot.commands.AutoStow;
import frc.robot.subsystems.ArmSub;
import frc.robot.util.Constants;
import frc.robot.util.Constants.ArmPositions;

public class Autos {
    private List<autoshit> autos = new ArrayList<autoshit>();
    private HashMap<String, autoshit> autoMap = new HashMap<String, autoshit>();
    private RobotContainer container;

    private SwerveAutoBuilder autoBuilder;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private StringTopic dblTopic = inst.getStringTopic("/Dashboard/auto selected");
    private StringSubscriber sub;

    private StringTopic chooserSelected = inst.getStringTopic("/SmartDashboard/Auto Selector/active");
    private StringSubscriber chooserSub;

    private BooleanTopic allianceSelected = inst.getBooleanTopic("/FMSInfo/IsRedAlliance");
    private BooleanSubscriber allianceSub = allianceSelected.subscribe(false);

    private SendableChooser<String> backup;

    private final boolean useShuffleBoard = true;
    
    class autoshit {
      public String name = "";
      public List<PathPlannerTrajectory> traj = new ArrayList<PathPlannerTrajectory>();
      public Command commandB = new WaitCommand(0);
      public Command commandR = new WaitCommand(0);
      public Trajectory fulltrajB = new PathPlannerTrajectory();
      public Trajectory fulltrajR = new PathPlannerTrajectory();
      public boolean DoPath = true;
      public autoshit(String name) {
        this.name = name;
      }
      public autoshit build(String hname, PathConstraints constraint, PathConstraints... constraints) {
        this.traj = PathPlanner.loadPathGroup(hname, false, constraint, constraints);
        this.commandB = this.commandB.andThen(autoBuilder.fullAuto(this.traj));
        this.commandR = this.commandR.andThen(autoBuilder.fullAuto(flipList(this.traj)));
        return this;
      } 

      public autoshit build(String hname) {
        PathConstraints pc;
        try {
          pc = PathPlanner.getConstraintsFromPath(hname);
        } catch (Throwable e){ // WISH THIS WOULD WORK, IT DOESN'T, STUPID JAVA SUCKS NUTS, HAVE TO CRASH INSTEAD OF IGNORING THE AUTO PATH
          System.out.println(e);
          DoPath = false;
          return this;
        }
        this.traj = PathPlanner.loadPathGroup(hname, false, pc);
        this.commandB = autoBuilder.fullAuto(this.traj);
        this.commandR = autoBuilder.fullAuto(flipList(this.traj));
        return this;
      }

      public autoshit add (Command end) {
        this.commandB = this.commandB.andThen(end);
        this.commandR = this.commandR.andThen(end);
        return this;
      }

      public void end() {
        if (!DoPath) return;

        for (int i = 0; i < traj.size(); ++i) {
            fulltrajB = concat(fulltrajB, traj.get(i));         
        }
        fulltrajR = flip(fulltrajB);
        this.commandR.getRequirements();
        autos.add(this);
      }

      private Trajectory concat (Trajectory a, PathPlannerTrajectory b) {
          List<State> States = new ArrayList<State>();

          for (int i = 0; i < a.getStates().size(); ++i) {
            var s = a.getStates().get(i);
            States.add(
                new State(
                    s.timeSeconds,
                    s.velocityMetersPerSecond,
                    s.accelerationMetersPerSecondSq,
                    s.poseMeters,
                    s.curvatureRadPerMeter));
          }

          for (int i = 0; i < b.getStates().size(); ++i) {
            var s = b.getState(i);
            var pose = new Pose2d(s.poseMeters.getTranslation(), s.holonomicRotation);
            States.add(
                new State(
                    s.timeSeconds + a.getTotalTimeSeconds(),
                    s.velocityMetersPerSecond,
                    s.accelerationMetersPerSecondSq,
                    pose,
                    s.curvatureRadPerMeter));
          }
          return new Trajectory(States);
        }
        
        private Trajectory flip (Trajectory blue) {
          if (blue.getStates().size() == 0) return blue;

          List<State> states = new ArrayList<State>();

          for (int i = 0; i < blue.getStates().size(); ++i) {
            State current = blue.getStates().get(i);
            Pose2d newPose = new Pose2d(new Translation2d((current.poseMeters.getTranslation().getX()) * -1 + 16.5, current.poseMeters.getTranslation().getY()), new Rotation2d(current.poseMeters.getRotation().getRadians() * -1 + Math.PI));
            states.add (
              new State(
                current.timeSeconds,
                current.velocityMetersPerSecond,
                current.accelerationMetersPerSecondSq,
                newPose,
                current.curvatureRadPerMeter
              )
            );
          }
          return new Trajectory(states);
        }

        private List<PathPlannerTrajectory> flipList(List<PathPlannerTrajectory> list) {
          List<PathPlannerTrajectory> end = new ArrayList<PathPlannerTrajectory>(list.size());
          for (PathPlannerTrajectory traj : list){
            List<State> states = new ArrayList<State>(traj.getStates().size());
            for (int i = 0; i < traj.getStates().size(); ++i) {
              PathPlannerState current = (PathPlannerState)traj.getStates().get(i);
              PathPlannerState b = new PathPlannerState();
              b.accelerationMetersPerSecondSq = current.accelerationMetersPerSecondSq;
              b.angularVelocityRadPerSec = current.angularVelocityRadPerSec;
              b.curvatureRadPerMeter = current.curvatureRadPerMeter;
              b.holonomicAngularVelocityRadPerSec = current.holonomicAngularVelocityRadPerSec;
              b.holonomicRotation = current.holonomicRotation.times(-1).plus(new Rotation2d(Math.PI));
              b.poseMeters = new Pose2d(new Translation2d((current.poseMeters.getTranslation().getX()) * -1 + 16.5, current.poseMeters.getTranslation().getY()), new Rotation2d(current.poseMeters.getRotation().getRadians() * -1 + Math.PI));
              b.timeSeconds = current.timeSeconds;
              b.velocityMetersPerSecond = current.velocityMetersPerSecond;

          states.add(b);  

        }
        PathPlannerTrajectory p = new PathPlannerTrajectory (states, traj.getMarkers(), traj.getStartStopEvent(), traj.getEndStopEvent(), traj.fromGUI);
        end.add(p);
      }
      return end;
        }

        
    }
    public autoshit getAuto() {
        if (useShuffleBoard)
          return autoMap.get(backup.getSelected()); 
        else
          return autoMap.get(sub.get());
    }

    public Command get() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? getAuto().commandB : getAuto().commandR;
    }

    public Trajectory getFullTraj() {
      return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? getAuto().fulltrajB : getAuto().fulltrajR;
    }

    public void setTraj() {
        container.s_Swerve.m_fieldSim.getObject("traj").setTrajectory(getFullTraj());
    }

    public void clearTraj() {
      container.s_Swerve.m_fieldSim.getObject("traj").setTrajectory(new Trajectory());
    }

    public boolean hasUpdated() {
      if (useShuffleBoard) {
        if (chooserSub.readQueueValues().length > 0) return true;
      } else {
        if(sub.readQueueValues().length > 0) return true;
      }
      if (allianceSub.readQueueValues().length > 0) return true;
      return false;
    }

    public void closeSubs() {
      if (chooserSub!=null)
        chooserSub.close();
      if (sub!=null)
        sub.close();
      if(allianceSub != null)
        allianceSub.close();
    }

    public void openSubs() {
      allianceSub = allianceSelected.subscribe(false);
      if (useShuffleBoard) 
        chooserSub = chooserSelected.subscribe("hi");
      else
        sub = dblTopic.subscribe("none");  
    }

    private void setToBegin(Pose2d pose) {
      container.s_Swerve.zeroGyro();
      //if (container.s_Swerve.getPose().getTranslation().getDistance(pose.getTranslation()) > 2)
         container.s_Swerve.resetOdometry(pose);

      //If Our Pose is off, reset Pose to at least run it a little correct. If cams don't work, this will save the auto!
    }

    public Autos(RobotContainer container){
        this.container = container;
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("goarm", new AdaptiveArmMovement(container.armSub, ArmPositions.MID_SCORE_ADAPTIVE));
        eventMap.put("stowarm", new AutoStow(container.armSub, container.gripper)); 
        eventMap.put("placehigh", new AutoPlace(container.armSub, container.gripper, ArmPositions.HIGH_SCORE_ADAPTIVE));
        eventMap.put("switchcone", new InstantCommand(() -> ArmSub.gamePiece = 0));
        eventMap.put("switchcube", new InstantCommand(() -> ArmSub.gamePiece = 1));
        eventMap.put("cubepickup", new AdaptiveArmMovement(container.armSub, ArmPositions.GROUND_PICKUP_ADAPTIVE, 1).alongWith(container.gripper.intake()) );
        eventMap.put("conepickup", new AdaptiveArmMovement(container.armSub, ArmPositions.GROUND_PICKUP_ADAPTIVE, 0).alongWith(container.gripper.intake()) );
        eventMap.put("placemid", new AutoPlace(container.armSub, container.gripper, ArmPositions.MID_SCORE_ADAPTIVE));
        eventMap.put("holdstow", new AdaptiveArmMovement(container.armSub, ArmPositions.STOWED_ADAPTIVE));
        eventMap.put("balance", new AutoBalanceStation(container.s_Swerve));
        eventMap.put("placelow", new AutoPlace(container.armSub, container.gripper, ArmPositions.GROUND_SCORE_ADAPTIVE));
        eventMap.put("shootcone", new AutoPlace(container.armSub, container.gripper, ArmPositions.SHOOT));

        autoBuilder = new SwerveAutoBuilder(
            container.s_Swerve::getPose, // Pose2d supplier
            this::setToBegin, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(4, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(3, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            container.s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap, // Event Map for adding commands.
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            container.s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );

        new autoshit("none").end();
        autoImportAutos();
        //new autoshit("Place High").add(eventMap.get("placehigh")).end();

        if (useShuffleBoard) {
          backup = new SendableChooser<String>();
          chooserSub = chooserSelected.subscribe("hi");
        } else
          sub = dblTopic.subscribe("none");  

        String[] names = new String[autos.size()];

        for (int i = 0; i < autos.size(); i++) {
            autoMap.put(autos.get(i).name, autos.get(i));
            names[i] = autos.get(i).name;

            if(useShuffleBoard) {
              backup.addOption(autos.get(i).name, autos.get(i).name);
            }
        }
        
        if(useShuffleBoard){
          backup.setDefaultOption("none", "none");
          SmartDashboard.putData("Auto Selector", backup);
        } else {
          SmartDashboard.putStringArray("Auto List", names);
        }
    }
    public void autoImportAutos() {
      //System.out.println("Working Directory = " + System.getProperty("user.dir"));
      //File[] files = new File("src/main/deploy/pathplanner/").listFiles(); // Simulation
      File[] files = new File("home/lvuser/deploy/pathplanner/").listFiles(); //Real Robot
      for (File f : files) {
        boolean vel = false;
        boolean accel = false;
        String pathname = f.getName().substring(0, f.getName().length() - 5);
        try (Scanner scanner = new Scanner(f)) {
          while (scanner.hasNextLine()) {
            final String lineFromFile = scanner.nextLine();
            if(lineFromFile.contains("maxVelocity") == true && lineFromFile.contains("null") == false) 
              vel = true;
            if (lineFromFile.contains("maxAcceleration") == true && lineFromFile.contains("null") == false)
              accel = true;
            if (vel && accel) break;
          }
          if (vel == false || accel == false)
            new autoshit(pathname).build(pathname, new PathConstraints(3, 3),new PathConstraints(3, 3)).end();
          else
            new autoshit(pathname).build(pathname).end();
        } catch (Exception e) {
          System.out.println(f.getName());
          System.out.println(e);
        } 
      } 
    }

    public void manuallyImportAutos() {
      new autoshit("Cable Protector Shoot 3").build("CableProtector", new PathConstraints(3, 3),new PathConstraints(3, 3)).end();
      new autoshit("Three Piece").build("ThreePieceCone",new PathConstraints(3, 3)).end();
      new autoshit("Two Piece Climb").build("FullAuto",new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3),new PathConstraints(3, 3), new PathConstraints(2, 2)).end();
      new autoshit("Middle One Piece").build("MiddleOnePiece",new PathConstraints(2, 2)).end();
      new autoshit("Middle Two Piece").build("MiddleTwoPiece",new PathConstraints(2, 2)).end();
      new autoshit("Cable Protector Climb").build("CableProtectorCharge",new PathConstraints(3, 3)).end();
      new autoshit("Cable Protector Double High").build("CableProtectorDoubleHigh",new PathConstraints(3, 3)).end();
      new autoshit("Test Path").build("Patha").end();
      new autoshit("Test").build("Test").end();
    }
}
