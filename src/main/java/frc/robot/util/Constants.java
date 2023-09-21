package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final class Swerve {
        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 10; //meters per second
        public static final double maxAngularVelocity = 16;

        public static final double driveGearRatio = (6.12 / 1.0);
        public static final double angleGearRatio = (12.8 / 1.0); 

        /* Drivetrain Constants */ 
        public static final double wheelX =  Units.inchesToMeters(17.5);
        public static final double wheelY =  Units.inchesToMeters(18.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
       
        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelX / 2.0, wheelY / 2.0),
                new Translation2d(wheelX / 2.0, -wheelY / 2.0),
                new Translation2d(-wheelX / 2.0, wheelY / 2.0),
                new Translation2d(-wheelX / 2.0, -wheelY / 2.0));

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final double angleOffset = 15.22;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final double angleOffset = 219.02;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final double angleOffset = 311.66;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final double angleOffset = 285.53;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final double stickDeadband = 0.06;

    public static final double stowedPosValue = 196;
    public static final double grndIntakePosValue = 180;

    public static final double grndScorePosValueCube = 166; 
    public static final double grndScorePosValueCone = 166; 

    public static final double midScorePosValueCube = 117;
    public static final double midScorePosValueCone = 90;

    public static final double highScorePosValueCube = 94; 
    public static final double highScorePosValueCone = 80; 

    public static final double shelfIntakePosValue = 94;

    public static final double coneUprightPosValue = 150;

    public static final class ArmRotationValues {

    public static final double armRotStow = 2;

    public static final double armRotForHighCone = 70;
    public static final double armRotForHighCube = 50;

    public static final double armRotForMidCone = 65;
    public static final double armRotForMidCube = 40;

    public static final double armRotForLowCone = 5;
    public static final double armRotForLowCube = 15;

    public static final double armRotForConePickup = 3;
    public static final double armRotForCubePickup = 3;

    public static final double armRotForShelfCone = 110;//120;
    public static final double armRotForShelfCube = 110;

    public static final double armRotRevHighCone = 141;
    public static final double armRotRevHighCube = 145;

    public static final double armRotRevMidCone = 135;
    public static final double armRotRevMidCube = 150;

    public static final double armRotRevLowCone = 30;
    public static final double armRotRevLowCube = 90;

    public static final double armRotRevConePickup = 180;
    public static final double armRotRevCubePickup = 170;
    
    public static final double armRotRevShelfCone = 95;//81;
    public static final double armRotRevShelfCube = 120;

    public static final double framePerimeter = 85;
    }

    public static final class ArmExtendValues {
        public static final double armExtendStow = 1000;

        public static final double armExtendConePickup = 7000;
        public static final double armExtendCubePickup = 7000;


        public static final double armExtendForHighCone = 48000;
        public static final double armExtendForHighCube = 48000;

        public static final double armExtendForMidCone = 44000;
        public static final double armExtendForMidCube = 20000;

        public static final double armExtendForLowCone = 0;
        public static final double armExtendForLowCube = 0;

        public static final double armExtendForConePickup = 7000;
        public static final double armExtendForCubePickup = 7000;

        public static final double armExtendForShelfCone = 0;//41000;
        public static final double armExtendForShelfCube = 41000;


        public static final double armExtendRevHighCone = 52000;
        public static final double armExtendRevHighCube = 39000;

        public static final double armExtendRevMidCone = 22000;
        public static final double armExtendRevMidCube = 0;

        public static final double armExtendRevLowCone = 0;
        public static final double armExtendRevLowCube = 0;

        public static final double armExtendRevShelfCone = 0;//43000;
        public static final double armExtendRevShelfCube = 43000;

        public static final double framePerimeter = 0;

    }

    public static final class JointRotationValues {
        public static final double JointRotStowCone = 500;
        public static final double JointRotStowCube = 1000;

        public static final double JointRotConePickup = 11000;
        public static final double JointRotCubePickup = 10000;
        

        public static final double JointRotForHighCone = 16000;
        public static final double JointRotForHighCube = 17000;

        public static final double JointRotForMidCone = 43000;
        public static final double JointRotForMidCube = 17000;

        public static final double JointRotForLowCone = 20000;
        public static final double JointRotForLowCube = 12000;

        public static final double JointRotForConePickup = 13000;
        public static final double JointRotForCubePickup = 8000;

        public static final double JointRotForShelfCone = 6500;//1500;
        public static final double JointRotForShelfCube = 9000;


        public static final double JointRotRevHighCone = 13000;
        public static final double JointRotRevHighCube = 2000;

        public static final double JointRotRevMidCone = 7500;
        public static final double JointRotRevMidCube = 4000;
  
        public static final double JointRotRevLowCone = 1000;
        public static final double JointRotRevLowCube = 0;

        public static final double JointRotRevShelfCone = 43200;//46000;
        public static final double JointRotRevShelfCube = 0;

        public static final double framePerimeter = 0;

    }

    public static enum ArmPositions{
        STOWED_ADAPTIVE,
        GROUND_PICKUP_ADAPTIVE, 
        GROUND_SCORE_ADAPTIVE, 
        MID_SCORE_CUBE, 
        MID_SCORE_CONE, 
        HIGH_SCORE_CUBE,
        HIGH_SCORE_CONE, 
        SHELF_PICKUP_ADAPTIVE, 
        MID_SCORE_ADAPTIVE,
        HIGH_SCORE_ADAPTIVE,
        CONE_UPRIGHT,
        FRAME_PERIMETER,
        SINGLE_INTAKE,
        UPRIGHT_CONE,
        SHOOT
    }

    public static class CANdle {
        public static final int id = 19;
    }

    public static class MAC_ADDRESSES {
        public static final byte[] COMP = new byte[]{
            // values are for comp -> 00:80:2f:35:b8:ca
            (byte)0x00,(byte)0xA4,(byte)0x75,(byte)0xED,(byte)0x67,(byte)0x8C
    };
        public static final byte[] SIM = new byte[]{
            // values are for comp -> 00:80:2f:35:b8:ca
            (byte)0xF4,(byte)0xA4,(byte)0x75,(byte)0xED,(byte)0x67,(byte)0x8C
    };
    }
}
