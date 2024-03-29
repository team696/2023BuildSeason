package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    PIDController pidController = new PIDController(0.02  , 0.00, 0);

    private Trigger leftJoy;
    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        pidController.setTolerance(1);
        pidController.enableContinuousInput(-180, 180);
        leftJoy = new JoystickButton(controller, 1);
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        if (leftJoy.getAsBoolean() != true){
            if (Math.abs(rAxis) > Constants.stickDeadband) {
                if (rAxis > 0)
                    rAxis = mapdouble(rAxis, Constants.stickDeadband, 1, 0, 1);
                else 
                    rAxis = mapdouble(rAxis, -Constants.stickDeadband, -1, 0, -1);
                
                //rAxis = Math.pow(rAxis,1.2) ;//* Math.signum(rAxis);
            }
            else{
                rAxis = 0;
            }
        } else {
            rAxis = pidController.calculate(s_Swerve.db_getYaw(), Math.abs(s_Swerve.db_getYaw()) < 90 ? 0 : 180);
        }

        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)),1);
        if (magnitude < Constants.stickDeadband) magnitude = 0;
        translation = new Translation2d(Math.pow(magnitude, 2), theta).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
