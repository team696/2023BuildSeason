package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
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
        pidController = new PIDController(0.1 , 0, 0);
    }

    @Override
    public void execute() {
        double value = 0;
        if (s_Swerve.getPitch() > 2.5) {
            value = 1;
        } 
        if (s_Swerve.getPitch() < -2.5) {
            value = -1;
        }
        double yAxis = pidController.calculate(value , 0);
        double xAxis = 0;
        double rAxis = 0;
    

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);
    }
    @Override
  public boolean isFinished() {
    // return pidController.atSetpoint();
    return false;
  }
}
