package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalanceStation extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
   
    PIDController pidController;

    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Driver control
     */
    public AutoBalanceStation(Swerve s_Swerve,  boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        pidController = new PIDController(0.8 , 0, 0);
        pidController.setTolerance(1);

        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = pidController.calculate(s_Swerve.getPitch(), 0);
        double xAxis = 0;
        double rAxis = 0;
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

        if (Math.abs(rAxis) > Constants.stickDeadband) {
            if (rAxis > 0){
                rAxis = mapdouble(rAxis, Constants.stickDeadband, 1, 0, 1);
            } else {
                rAxis = mapdouble(rAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            rAxis = 0;
        }

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
    @Override
  public boolean isFinished() {
    // return pidController.atSetpoint();
    return false;
  }
}
