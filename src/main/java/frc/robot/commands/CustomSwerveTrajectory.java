package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class CustomSwerveTrajectory extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    PIDController xPID;
    PIDController yPID;
    PIDController rotPID;

    double goalX;
    double goalY;
    double goalRot;
     
    double maxSpeed;
    double maxAngularVelocity;

  
    /**
     * Driver control
     */
    public CustomSwerveTrajectory(Swerve s_Swerve, double goalX, double goalY, double goalRot, double maxSpeed, double maxAngularVelocity) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        xPID = new PIDController(/* 0.65 */2, 0.0, 0.05);
        yPID = new PIDController(/* 0.65 */2, 0.0, 0.05);
        rotPID = new PIDController(0.003, 0.0, 0);

        xPID.setTolerance(0.025);
        yPID.setTolerance(0.025);
        rotPID.setTolerance(1);
        rotPID.enableContinuousInput(-180, 180);
        


       this.goalX = goalX;
       this.goalY = goalY;
       this.goalRot = goalRot;
       this.maxSpeed = maxSpeed;
       this.maxAngularVelocity = maxAngularVelocity;
    }

    @Override
    public void execute() {
        double yAxis = xPID.calculate(s_Swerve.getPose().getX(), goalX);
        double xAxis = yPID.calculate(s_Swerve.getPose().getY(), goalY);
        double rAxis = rotPID.calculate(s_Swerve.getPose().getRotation().getDegrees(), goalRot);
        
        /* Deadbands */

    

        translation = new Translation2d(yAxis, xAxis).times(maxSpeed);
        rotation = rAxis * maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, false);
    }

    @Override
  public boolean isFinished() {
    // return swerveControllerCommand.isFinished();
      return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint();
  }

}
