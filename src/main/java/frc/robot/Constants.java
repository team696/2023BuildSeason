package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.GlobalVariables.ArmPositions;

public final class Constants {
    public static final double stickDeadband = 0.06;

    public static final double rotatePid_FF = 0.09;
    public static final double rotatePid_P = /* 0.017 */ 0.015;
    public static final double rotatePid_I = /* 0.0002 */ 0.0;
    public static final double rotatePid_D = /* 0.003 */0.002125;
    public static final double rotatePid_Tol = 1;

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

    public static final double armRotStow = 5;

    public static final double armRotForHighCone = 70;
    public static final double armRotForHighCube = 50;

    public static final double armRotForMidCone = 65;
    public static final double armRotForMidCube = 40;

    public static final double armRotForLowCone = 5;
    public static final double armRotForLowCube = 15;

    public static final double armRotForConePickup = 3;
    public static final double armRotForCubePickup = 3;

    public static final double armRotForShelfCone = 105;//120;
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
        public static final double armExtendStow = 500;

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
        public static final double JointRotForMidCube = 17000
        ;

        public static final double JointRotForLowCone = 20000;
        public static final double JointRotForLowCube = 12000;

        public static final double JointRotForConePickup = 11000;
        public static final double JointRotForCubePickup = 8000;

        public static final double JointRotForShelfCone = 8500;//1500;
        public static final double JointRotForShelfCube = 9000;


        public static final double JointRotRevHighCone = 12000;
        public static final double JointRotRevHighCube = 2000;

        public static final double JointRotRevMidCone = 7500;
        public static final double JointRotRevMidCube = 4000;
  
        public static final double JointRotRevLowCone = 1000;
        public static final double JointRotRevLowCube = 0;

        public static final double JointRotRevShelfCone = 43200;//46000;
        public static final double JointRotRevShelfCube = 0;

        public static final double framePerimeter = 0;

    }




    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */ 
        public static final double trackWidth =  Units.inchesToMeters(17.5) /*  0.521 */;
        public static final double wheelBase =  Units.inchesToMeters(18.5)     /* 0.622 */;
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.05;
        public static final double closedLoopRamp =  0.61 ;

        public static final double driveGearRatio = (6.12 / 1.0); //8:14:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 10; //meters per second
        public static final double maxAngularVelocity = 16;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = /* 2 */5;
            public static final int angleMotorID = /* 1 */4;
            public static final int canCoderID = /* 3 */6;
            public static final double angleOffset = /* 274; */  15.22;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = /* 11 */11;
            public static final int angleMotorID = /* 10 */10;
            public static final int canCoderID = /* 12 */12;
            public static final double angleOffset = /* 240 ; */ 219.02;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = /* 5 */2;
            public static final int angleMotorID = /* 4 */1;
            public static final int canCoderID = /* 6 */3;
            public static final double angleOffset = /* 319; */ 311.66;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = /* 8 */8;
            public static final int angleMotorID = /* 7 */7;
            public static final int canCoderID = /* 9 */9;
            public static final double angleOffset = /* 305 ; */ 285.53;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.05;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond/2, kMaxAngularSpeedRadiansPerSecondSquared/2);

                // [AprilTag #] [Left To Right] [ Bottom To Top]
        public static final double RobotPositions[][][][] = {
            { // Leftmost Tag
                { {2.1, 5.0}, {2.3, 5.0}, {1.8, 5.0} }, //Left scoring
                { {2.1, 4.46}, {2.3, 4.46}, {1.8, 4.46} }, //Middle Scoring
                { {2.1, 3.88}, {2.3, 3.88}, {1.8, 3.88} },  //Right Scoring
                //  low             mid          high 
            }, 
            { //Middle Tag
                { {2.1, 3.31}, {2.3, 3.31}, {1.8, 3.31} }, 
                { {2.1, 2.85 }, {2.3, 2.70 }, { 1.8, 2.85 } },
                { {2.1, 2.25}, {2.3, 2.25}, {1.8, 2.25} },
            },
            { // Right Tag
                { {2.1, 1.72}, {2.3, 1.72}, {1.8, 1.72} }, 
                { {2.1, 1.15}, {2.3, 1.15}, {1.8, 1.15} },
                { {2.1, 0.5}, {2.3, 0.5}, {1.8, 0.5} },
            }
        };
        // [0 is Cone, 1 is Cube] [Low, Mid, High]
       

        public static final double RobotPositionsRed[][][][] = {
            { // Leftmost Tag
                { {14.44, 0.5}, {14.24, 0.5}, {14.74, 0.5} },
                { {14.44, 1.15}, {14.24, 1.15}, {14.74, 1.15} },
                { {14.44, 1.72}, {14.24, 1.72}, {14.74, 1.72} }, 

                //  low             mid          high 
            }, 
            { //Middle Tag
                { {14.44, 2.25}, {14.24, 2.25}, {14.74, 2.25} },
                { {14.44, 2.85 }, {14.24, 2.70 }, { 14.74, 2.85 } },   
                { {14.44, 3.31}, {14.24, 3.31}, {14.74, 3.31} }, 

            },
            { // Right Tag
                { {14.44, 3.88}, {14.24, 3.88}, {14.8, 3.88} },  //Right Scoring
                { {14.44, 4.46}, {14.24, 4.46}, {14.8, 4.46} }, //Middle Scoring
                { {14.44, 5.0}, {14.24, 5.0}, {14.8, 5.0} }, //Left scoring

            }
        };
      }

         public static final double ArmPositions[][] = {
            { grndScorePosValueCone,midScorePosValueCone,highScorePosValueCone }, 
            { grndScorePosValueCube,midScorePosValueCube,highScorePosValueCube }
        };

      static class FieldConstants {
        static final double length = Units.feetToMeters(54 + 3.25/12);
        static final double width = Units.feetToMeters(26 + 3.5/12);
    }

    static class VisionConstants {
        static final Transform3d robotToCam1 =
                new Transform3d(
                        new Translation3d(-Units.feetToMeters(0.95096), -Units.feetToMeters(0.924031), Units.feetToMeters(1.79878)),
                        new Rotation3d(
                                0, 0,
                                Math.toRadians(-45))); 

        static final Transform3d robotToCam2 =
                new Transform3d(
                        new Translation3d(-Units.feetToMeters(0.95096), Units.feetToMeters(0.924031), Units.feetToMeters(1.79878)),
                        new Rotation3d(
                                0, 0,
                                Math.toRadians(45)));
        static final String camera1Name = "RearCam";
        static final String camera2Name = "FrontCam";
        static final String frontCamName = "HD_USB_Camera";
    }
    
    public static class CANdle {
        public static final int id = 19;
    }
}
