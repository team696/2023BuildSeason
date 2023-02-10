// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class GlobalVariables {
    
    public static boolean gamePiece;
    public static int portionSelect;
    public static int rowSelect;
    public static int columnSelect;

    public static enum StationSelect{
        CLOSE_RAMP, LEFT_SHELF, LEFT_RAMP, RIGHT_SHELF, RIGHT_RAMP
    }

    public static StationSelect stationSelect = StationSelect.LEFT_RAMP;


    public static enum ArmPositions{
        STOWED, GROUND_PICKUP, GROUND_SCORE, MID_SCORE, HIGH_SCORE, SHELF_PICKUP
    }

    public static ArmPositions armPosition;

    

}

