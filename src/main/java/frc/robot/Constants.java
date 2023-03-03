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


    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */ 
        public static final double trackWidth =  Units.inchesToMeters(24) /*  0.521 */;
        public static final double wheelBase =  Units.inchesToMeters(25)     /* 0.622 */;
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.2;
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
            public static final double angleOffset =    /* 7.9 */ 304;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = /* 11 */11;
            public static final int angleMotorID = /* 10 */10;
            public static final int canCoderID = /* 12 */12;
            public static final double angleOffset =   /* 74  */218 ;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = /* 5 */2;
            public static final int angleMotorID = /* 4 */1;
            public static final int canCoderID = /* 6 */3;
            public static final double angleOffset = /* 307 */312;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = /* 8 */8;
            public static final int angleMotorID = /* 7 */7;
            public static final int canCoderID = /* 9 */9;
            public static final double angleOffset = /* 146 */ 289 ;
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
        static final String camera1Name = "OV9281-01";
        static final String camera2Name = "Cam_2";
        static final String frontCamName = "HD_USB_Camera";
    }
    
    public static class CANdle {
        public static final int id = 19;
    }
}
