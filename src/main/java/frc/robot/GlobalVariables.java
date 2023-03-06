// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class GlobalVariables {
    
    public static int gamePiece = 0;
    public static int tag;
    public static int rowSelect;
    public static int height;
    public static double armTrim;

    public static enum StationSelect{
        CLOSE_RAMP, LEFT_SHELF, LEFT_RAMP, RIGHT_SHELF, RIGHT_RAMP
    }

    public static StationSelect stationSelect = StationSelect.LEFT_RAMP;


    public static enum ArmPositions{
        STOWED,
        GROUND_PICKUP, 
        GROUND_SCORE, 
        MID_SCORE_CUBE, 
        MID_SCORE_CONE, 
        HIGH_SCORE_CUBE,
        HIGH_SCORE_CONE, 
        SHELF_PICKUP, 
        MID_SCORE_ADAPTIVE,
        HIGH_SCORE_ADAPTIVE,
        CONE_UPRIGHT
    }

    public static ArmPositions armPosition;

    

}

