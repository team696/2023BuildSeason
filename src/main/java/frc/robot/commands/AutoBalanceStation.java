package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalanceStation extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private Swerve s_Swerve;
   
    PIDController pidController;
    /**
     * Driver control
     */
    public AutoBalanceStation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve ;
        addRequirements(s_Swerve);
        pidController = new PIDController(0.0975, 0, 0);
    }
    
    @Override
    public void execute() {
        double yvalue = 0;
        if (s_Swerve.getPitch() > 2.5) {
            yvalue = 1;
        } 
        if (s_Swerve.getPitch() < -2.5) {
            yvalue = -1;
        }
        double xvalue = 0;
        if (s_Swerve.getRoll() > 2.5) {
            xvalue = -1;
        }
        if (s_Swerve.getRoll() < -2.5) {
            xvalue = 1;
        }
        double yAxis = pidController.calculate(yvalue , 0);
        double xAxis = pidController.calculate(xvalue, 0);
        double rAxis = 0;
    

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);
    }
    @Override
  public boolean isFinished() {
    return false;
  }
}
