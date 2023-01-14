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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModulePosition[] mSwerveModulePositions;
    private final Field2d m_fieldSim = new Field2d();
    public PhotonCameraWrapper pcw;
    public AHRS gyro;
    private final SwerveModule m_frontLeft = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    private final SwerveModule m_frontRight = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    private final SwerveModule m_backLeft = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    private final SwerveModule m_backRight = new SwerveModule(3, Constants.Swerve.Mod3.constants);
    public SwerveModule[] mSwerveMods = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

private  SwerveDrivePoseEstimator m_poseEstimator;

  

    public Swerve() {
        pcw = new PhotonCameraWrapper();
        gyro  = new AHRS();
        
        zeroGyro();
        SmartDashboard.putData("Field", m_fieldSim);
 
        mSwerveModulePositions =new SwerveModulePosition[] {
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

        m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
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
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), mSwerveModulePositions, pose);
    }

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
        if (gyro.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
          }
           return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }

    

    @Override
    public void periodic(){
        // m_poseEstimator.
         swerveOdometry.update(getYaw(), mSwerveModulePositions);  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}