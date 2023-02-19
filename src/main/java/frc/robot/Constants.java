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

public final class Constants {
    public static final double stickDeadband = 0.06;

    public static final double rotatePid_FF = 0.09;
    public static final double rotatePid_P = /* 0.017 */ 0.015;
    public static final double rotatePid_I = /* 0.0002 */ 0.0;
    public static final double rotatePid_D = /* 0.003 */0.002125;
    public static final double rotatePid_Tol = 1;

    public static final double stowedPosValue = 230;
    public static final double grndIntakePosValue = 216;
    public static final double grndScorePosValue = 207; 
    public static final double midScorePosValue = 130;
    public static final double highScorePosValue = 113; 
    public static final double shelfIntakePosValue = 37;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */ 
        public static final double trackWidth =  Units.inchesToMeters(24) /*  0.521 */;
        public static final double wheelBase =  Units.inchesToMeters(24)     /* 0.622 */;
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp =0.2;
        public static final double closedLoopRamp = 0.5;

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
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

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
            public static final double angleOffset =    /* 7.9 */ 275;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = /* 11 */11;
            public static final int angleMotorID = /* 10 */10;
            public static final int canCoderID = /* 12 */12;
            public static final double angleOffset =   /* 74  */240 ;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = /* 5 */2;
            public static final int angleMotorID = /* 4 */1;
            public static final int canCoderID = /* 6 */3;
            public static final double angleOffset = /* 307 */318;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = /* 8 */8;
            public static final int angleMotorID = /* 7 */7;
            public static final int canCoderID = /* 9 */9;
            public static final double angleOffset = /* 146 */ 305  ;
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
                kMaxAngularSpeedRadiansPerSecond/8, kMaxAngularSpeedRadiansPerSecondSquared/2);

                // [AprilTag #] [Left To Right] [ Bottom To Top]
        public static final double RobotPositions[][][][] = {
            { // Leftmost Tag
                { {1.74, 5.06}, {2.1, 5.06}, {1.74, 5.06} }, //Left scoring
                { {1.74, 4.46}, {2.1, 4.46}, {1.74, 4.46} }, //Middle Scoring
                { {1.74, 3.87}, {2.1, 3.87}, {1.74, 3.87} },  //Top Scoring
            }, 
            { //Middle Tag
                { {1.74, 3.28}, {2.1, 3.28}, {1.74, 3.28} }, 
                { {1.74, 2.7 }, {2.1, 2.70}, {1.74, 2.70} },
                { {1.74, 2.16}, {2.1, 2.16}, {1.74, 2.16} },
            },
            { // Right Tag
                { {1.74, 1.59}, {2.1, 1.59}, {1.74, 1.59} }, 
                { {1.74, 1.37}, {2.1, 1.37}, {1.74, 1.37} },
                { {1.74, 0.58}, {2.1, 0.58}, {1.74, 0.58} },
            }
        };
        // [0 is Cone, 1 is Cube] [Low, Mid, High]
        public static final double ArmPositions[][] = {
            { 1,1,1 }, 
            { 1,1,1 }
        };
      }

        

      static class FieldConstants {
        static final double length = Units.feetToMeters(54 + 3.25/12);
        static final double width = Units.feetToMeters(26 + 3.5/12);
    }

    static class VisionConstants {
        static final Transform3d robotToCam1 =
                new Transform3d(
                        new Translation3d(-Units.feetToMeters(0.95096), Units.feetToMeters(0.924031), Units.feetToMeters(1.79878)),
                        new Rotation3d(
                                0, 0,
                                Math.toRadians(45))); 

        static final Transform3d robotToCam2 =
                new Transform3d(
                        new Translation3d(-Units.feetToMeters(0.95096), -Units.feetToMeters(0.924031), Units.feetToMeters(1.79878)),
                        new Rotation3d(
                                0, 0,
                                Math.toRadians(-45)));
        static final String camera1Name = "OV9281-01";
        static final String camera2Name = "OV9281-02";
    }
}
