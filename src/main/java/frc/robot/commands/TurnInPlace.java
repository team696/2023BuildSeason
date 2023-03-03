package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TurnInPlace extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    double goal;
    
    private Swerve s_Swerve;
   
    PIDController pidController;

    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Driver control
     */
    public TurnInPlace(Swerve s_Swerve,  boolean fieldRelative, boolean openLoop, double goal) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        pidController = new PIDController(0.003  , 0.00, 0);
        pidController.setTolerance(1);

        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.goal = goal;
    }

    @Override
    public void execute() {
        double yAxis = 0;
        double xAxis = 0;
        double rAxis = pidController.calculate(s_Swerve.getAprilTagEstPosition().getRotation().getDegrees(), goal);
        System.out.println("AUTO BALANCING ");
        
        /* Deadbands */

        if (Math.abs(yAxis) > Constants.stickDeadband) {
            if (yAxis > 0){
                yAxis = mapdouble(yAxis, Constants.stickDeadband, 1, 0, 1);
            } else {
                yAxis = mapdouble(yAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            yAxis = 0;
        }
        
        if (Math.abs(xAxis) > Constants.stickDeadband) {
            if (xAxis > 0){
                xAxis = mapdouble(xAxis, Constants.stickDeadband, 1, 0, 1);
            } else {
                xAxis = mapdouble(xAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            xAxis = 0;
        }

        // if (Math.abs(rAxis) > Constants.stickDeadband) {
        //     if (rAxis > 0){
        //         rAxis = mapdouble(rAxis, Constants.stickDeadband, 1, 0, 1);
        //     } else {
        //         rAxis = mapdouble(rAxis, -Constants.stickDeadband, -1, 0, -1);
        //     }
        // }
        // else{
        //     rAxis = 0;
        // }

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
    @Override
    public boolean isFinished() {
      // return swerveControllerCommand.isFinished();
if (s_Swerve.getAprilTagEstPosition().getRotation().getDegrees() < (goal+3) && s_Swerve.getAprilTagEstPosition().getRotation().getDegrees() > (goal -3)){
  return true;
}
else{
  return false;
}
    }
    
}
