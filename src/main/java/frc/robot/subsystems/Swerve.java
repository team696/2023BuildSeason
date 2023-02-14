package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.PhotonCameraWrapper;
// import frc.robot.PhotonCameraWrapper;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModulePosition[] mSwerveModulePositions;
    private final Field2d m_fieldSim = new Field2d();
    public PhotonCameraWrapper pcw;
    public AHRS gyro;
    public ProfiledPIDController rotatePID;

    private  SwerveModule m_frontLeft = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    private  SwerveModule m_frontRight = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    private  SwerveModule m_backLeft = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    private  SwerveModule m_backRight = new SwerveModule(3, Constants.Swerve.Mod3.constants);
    public SwerveModule[] mSwerveMods = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

private  SwerveDrivePoseEstimator m_poseEstimator;

  

    public Swerve() {
        pcw = new PhotonCameraWrapper();
        gyro  = new AHRS();
        
        
        // zeroGyro();
        SmartDashboard.putData("Field", m_fieldSim);

        rotatePID = new ProfiledPIDController(
            Constants.rotatePid_P,
            Constants.rotatePid_I,
            Constants.rotatePid_D, new TrapezoidProfile.Constraints(1, 3));

    rotatePID.setTolerance(Constants.rotatePid_Tol);
 
        mSwerveModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, m_frontRight.getCanCoder()),
            new SwerveModulePosition(0, m_frontLeft.getCanCoder()),
            new SwerveModulePosition(0, m_backRight.getCanCoder()),
            new SwerveModulePosition(0, m_backLeft.getCanCoder())
          };


        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), mSwerveModulePositions);

        m_poseEstimator = new SwerveDrivePoseEstimator( Constants.Swerve.swerveKinematics, getYaw(), mSwerveModulePositions, getPose());
    }
    public void updateOdometry() {
        m_poseEstimator.update(getYaw(), mSwerveModulePositions);
               
        Pair<Pose2d, Double> result = pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
        var camPose = result.getFirst();
        var camPoseObsTime = result.getSecond();
        
        if (camPose != null) {
            m_poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
            m_fieldSim.getObject("Cam Est Pos").setPose(camPose);
        } else {
            m_fieldSim.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
        }

        // m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
        m_fieldSim.setRobotPose(getPose());
    
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
    //  return    m_poseEstimator.update(getYaw(), mSwerveModulePositions);
        // updateOdometry();       
        return swerveOdometry.getPoseMeters();
    //   return  m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        // swerveOdometry.resetPosition(getYaw(), mSwerveModulePositions, pose);
        swerveOdometry.resetPosition(getYaw(), mSwerveModulePositions, new Pose2d());
    }

    public void normalizeOdometry(){
        swerveOdometry.resetPosition(getYaw(), mSwerveModulePositions, m_poseEstimator.getEstimatedPosition());
    }

    public Pose2d getAprilTagEstPosition(){
        return m_poseEstimator.getEstimatedPosition();
    }

    // public void relocalizeOdometry(){
    //     swerveOdometry.r
    // }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        // if (gyro.isMagnetometerCalibrated()) {
            // return Rotation2d.fromDegrees( gyro.getYaw() );
        //   }
           return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

   
//     public double limelightOffset(){

//         // float Kp = -0.1f;
// double min_command = 0.09;


//         double heading_error = pcw.horOffset();
//         double steering_adjust = 0.0;
//         if ((pcw.horOffset()+1) > 2)
//         {
//                 steering_adjust = /* Kp*heading_error */ - min_command;
//         }
//         else if ((pcw.horOffset()+1) < 0)
//         {
//                 steering_adjust =/*  Kp*heading_error + */ min_command;
//         }

//         if(pcw.horOffset() > (0.01) ||
//         pcw.horOffset() <  -(0.01)){
//       return (rotatePID.calculate(pcw.horOffset(), 1) + steering_adjust);
//         }
//         else {
//         return 0;
//         }
// }

// public double joyControlUntilLock(double joystick){

//       if(pcw.hasTarget()){
//               return limelightOffset();
//       }
//       else {
//               return joystick;
//       }
// }

    
    

    @Override
    public void periodic(){
        // updateOdometry();
        // m_poseEstimator.
         swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
          });  


           SmartDashboard.putNumber("lmao X", getPose().getX());
        SmartDashboard.putNumber("lmao Y", getPose().getY());
        SmartDashboard.putNumber("Gyro FH", gyro.getFusedHeading());
        SmartDashboard.putNumber("Gyro YAW", gyro.getYaw());
        SmartDashboard.putNumber("Gyro RawGyroZ", gyro.getRawGyroZ());

        SmartDashboard.putBoolean("Gyro IsRotating", gyro.isRotating());
        SmartDashboard.putNumber(" Gyro AngleAdjustment ", gyro.getAngleAdjustment());
        SmartDashboard.putNumber("Gyro Rotation2d", gyro.getRotation2d().getDegrees());
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        SmartDashboard.putNumber("Gyro Temp", gyro.getTempC());
        SmartDashboard.putNumber("Gyro RawMagZ", gyro.getRawMagZ());
        SmartDashboard.putBoolean("Gyro IsMagnometerCalibrated", gyro.isMagnetometerCalibrated());
        SmartDashboard.putBoolean("Gyro isMagneticDisturbance", gyro.isMagneticDisturbance());




        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}