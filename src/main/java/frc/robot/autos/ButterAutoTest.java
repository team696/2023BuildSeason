// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CustomSwerveTrajTrap;
import frc.robot.commands.CustomSwerveTrajectory;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ButterAutoTest extends SequentialCommandGroup {
  Swerve swerve;
  /** Creates a new ButterAutoTest. */
  public ButterAutoTest(Swerve swerve) {
    this.swerve = swerve;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
       new CustomSwerveTrajectory(swerve, 6.86, 4.56, 0, 0.3, 4),
      new CustomSwerveTrajectory(swerve, 3 , 4.97, 180, 0.3, 4),
      new CustomSwerveTrajectory(swerve, 6.86, 4.56, 0, 0.3, 4),
      new CustomSwerveTrajectory(swerve, 3 , 4.97, 180, 0.3, 4)
 
/* new CustomSwerveTrajTrap(swerve, 6.86, 4.56, 0, 0.1, 6),
new CustomSwerveTrajTrap(swerve, 3 , 4.97, 180, 0.1, 6),
new CustomSwerveTrajTrap(swerve, 6.86, 4.56, 0, 0.1, 6),
new CustomSwerveTrajTrap(swerve, 3 , 4.97, 180, 0.1, 6)
 */
    );
  }
}
