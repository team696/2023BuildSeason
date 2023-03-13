package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CustomSwerveTrajTrap extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    ProfiledPIDController xPID;
    ProfiledPIDController yPID;
    PIDController rotPID;

    double goalX;
    double goalY;
    double goalRot;
     
    double maxSpeed;
    double maxAngularVelocity;

    TrapezoidProfile.Constraints constraints;

  
    /**
     * Driver control
     */
    public CustomSwerveTrajTrap(Swerve s_Swerve, double goalX, double goalY, double goalRot, double maxSpeed, double maxAngularVelocity) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
       this.goalX = goalX;
       this.goalY = goalY;
       this.goalRot = goalRot;
       this.maxSpeed = maxSpeed;
       this.maxAngularVelocity = maxAngularVelocity;


        constraints = new TrapezoidProfile.Constraints(maxSpeed, 1);
        xPID = new ProfiledPIDController(/* 0.65 */2, 0.0, 0.02,constraints );
        yPID = new ProfiledPIDController(/* 0.65 */2, 0.0, 0.02, constraints);
        rotPID = new PIDController(0.003, 0.0, 0);

        xPID.setTolerance(0.025);
        yPID.setTolerance(0.025);
          rotPID.setTolerance(1);
        rotPID.enableContinuousInput(-180, 180);

      




    }
    @Override
  public void initialize() {

  xPID.reset(s_Swerve.getPose().getX()); 
  yPID.reset(s_Swerve.getPose().getY());
        
    xPID.setGoal(new TrapezoidProfile.State(goalX, 0));
    yPID.setGoal(new TrapezoidProfile.State(goalY, 0));

  }


    @Override
    public void execute() {
        double yAxis = xPID.calculate(s_Swerve.getPose().getX());
        double xAxis = yPID.calculate(s_Swerve.getPose().getY());
        double rAxis = rotPID.calculate(s_Swerve.getPose().getRotation().getDegrees(), goalRot);
        
        /* Deadbands */

    

        translation = new Translation2d(yAxis, xAxis).times(11.5);
        rotation = rAxis * maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, false);
    }

    @Override
  public boolean isFinished() {
    // return swerveControllerCommand.isFinished();
      return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }

}
