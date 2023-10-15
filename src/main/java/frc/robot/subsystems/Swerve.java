package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.util.Constants;
import frc.robot.util.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Swerve extends SubsystemBase {
    public final Field2d m_fieldSim = new Field2d();
   
    private AHRS gyro;
    private ProfiledPIDController rotatePID;
    private PIDController aimPID;

    private final SwerveModule m_frontLeft = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    private final SwerveModule m_frontRight = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    private final SwerveModule m_backLeft = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    private final SwerveModule m_backRight = new SwerveModule(3, Constants.Swerve.Mod3.constants);
    private final SwerveModule[] mSwerveMods = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };
    private SwerveModulePosition[] SwervePositions = new SwerveModulePosition[4];
    private SwerveDrivePoseEstimator m_poseEstimator;

    private PhotonCamera cams[]; // Next Time Move Cams into seperate class, use threads to split load??????????
    private PhotonPoseEstimator estimators[];

    public double gyroOffset = 0;

    public boolean UseCameras = true;

    public Swerve() {
        gyro  = new AHRS();
        aimPID = new PIDController(0.05, 0, 0);
        aimPID.setTolerance(0.5);

        rotatePID = new ProfiledPIDController(
            0.010,
            0,
            0.001, new TrapezoidProfile.Constraints(1, 3));

        rotatePID.setTolerance(0.001);
 
        for (int i = 0; i < 4; ++i) {
            SwervePositions[i] = mSwerveMods[i].getPosition();
        }

        m_poseEstimator = new SwerveDrivePoseEstimator( Constants.Swerve.swerveKinematics, getYaw(), SwervePositions, new Pose2d(1.0, 1.0, getYaw()), VecBuilder.fill(0.1, 0.1, 0.05), VecBuilder.fill(0.7, 0.7, 0.6));
            
        AprilTagFieldLayout aprilTagFieldLayout;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            
            cams = new PhotonCamera[]{
                new PhotonCamera("B"),
                new PhotonCamera("C"),
                new PhotonCamera("A"),
            };
            
            estimators = new PhotonPoseEstimator[] {
                new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, cams[0], new Transform3d(new Translation3d(-0.28, -0.0432, 0.0),  new Rotation3d(0.0, Math.toRadians(-11.0), Math.toRadians(-45.0 + 180)))),
                new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, cams[1], new Transform3d(new Translation3d(-0.262, 0.0, 0.0),  new Rotation3d(0.0, Math.toRadians(-11.0), Math.toRadians(180)))),
                new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP, cams[2], new Transform3d(new Translation3d(-0.28, 0.0432, 0.0), new Rotation3d(0.0, Math.toRadians(-11.0), Math.toRadians(45.0 + 180))))
            };
            for (PhotonPoseEstimator estimator : estimators) 
            {
                estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            }
        } catch (Exception e) {
            System.out.println(e);
        }

        SmartDashboard.putData(this);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getPose().getRotation().plus(new Rotation2d((DriverStation.getAlliance() == DriverStation.Alliance.Red ? Math.PI : 0)))
                                ) : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  
    
    public void Drive(ChassisSpeeds c, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(c);
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
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getYaw(), SwervePositions, pose);
    }

    public void resetOdometry(){
       resetOdometry(new Pose2d());
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
        resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d((DriverStation.getAlliance() == DriverStation.Alliance.Red ? Math.PI : 0))));
        gyroOffset = 0;
    }


    public double db_getYaw() {
        return (-1 * gyro.getYaw() + 180 + gyroOffset) % 360 - 180;
    }

    public Rotation2d getYaw() {
           return Rotation2d.fromDegrees(db_getYaw());
    }

    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void periodic(){
        for (int i = 0; i < 4; ++i) {
            SwervePositions[i] = mSwerveMods[i].getPosition();
        }

        m_poseEstimator.update(getYaw(), SwervePositions);
        
        if (UseCameras) {
            for (int i = 0; i < 3; i ++) {
                PhotonPoseEstimator estimator = estimators[i];
                final Optional<EstimatedRobotPose> est = estimator.update();
                if (est.isPresent()) {
                    boolean checkAmb = true;
                    final EstimatedRobotPose estimatedPoseA = est.get();
                    for (PhotonTrackedTarget t : estimatedPoseA.targetsUsed) {
                        if (t.getPoseAmbiguity() == -1 || t.getPoseAmbiguity() > 0.2  )
                            checkAmb = false;
                        //Transform3d offsetFromTarget = t.getBestCameraToTarget();
                        //if (Math.sqrt(Math.pow(offsetFromTarget.getX(),2) + Math.pow(offsetFromTarget.getY(),2)) > 4)
                            //checkAmb = false;
                    }
                    if(checkAmb)
                        m_poseEstimator.addVisionMeasurement(estimatedPoseA.estimatedPose.toPose2d(), estimatedPoseA.timestampSeconds); 
                }
            } 
        }
        if (Robot.robotNum != -1) {
            m_fieldSim.setRobotPose(getPose());
        } 
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        for(SwerveModule mod : mSwerveMods){
            builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.db_getCanCoder(), null);
        }
        builder.addDoubleProperty("Gyro", ()->db_getYaw(), null);
        SmartDashboard.putData("Field", m_fieldSim);
    }
}