package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLockToGamePiece extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    // private Joystick controller;
    private int translationAxis;
    private int strafeAxis;

    private int pipeline;
    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Driver control
     */
    public AutoLockToGamePiece(Swerve s_Swerve,  int translationAxis, int strafeAxis,  boolean fieldRelative, boolean openLoop, int pipeline) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        // this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.pipeline = pipeline;
    }

    @Override
    public void execute() {
        double yAxis = 0.3;
        double xAxis = 0;
        double rAxis = s_Swerve.frontCamOffset(pipeline);
        
        
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

        if (Math.abs(rAxis) > 0.01) {
            if (rAxis > 0){
                rAxis = mapdouble(rAxis, 0.01, 1, 0, 1);
            } else {
                rAxis = mapdouble(rAxis, -0.01, -1, 0, -1);
            }
        }
        else{
            rAxis = 0;
        }

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, false, false);
    }
}
