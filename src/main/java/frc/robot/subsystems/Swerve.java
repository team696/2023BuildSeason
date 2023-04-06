package frc.robot.subsystems;



import com.kauailabs.navx.frc.AHRS;
import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.PhotonCameraWrapper;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModulePosition[] mSwerveModulePositions;
    public Field2d m_fieldSim = new Field2d();
    public PhotonCameraWrapper pcw;
    public AHRS gyro;
    public ProfiledPIDController rotatePID;
    public PIDController aimPID;

    private  SwerveModule m_frontLeft = new SwerveModule(0, Constants.Swerve.Mod0.constants);
    private  SwerveModule m_frontRight = new SwerveModule(1, Constants.Swerve.Mod1.constants);
    private  SwerveModule m_backLeft = new SwerveModule(2, Constants.Swerve.Mod2.constants);
    private  SwerveModule m_backRight = new SwerveModule(3, Constants.Swerve.Mod3.constants);
    public SwerveModule[] mSwerveMods = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

    public static SendableChooser<Integer> AprilTagGrid = new SendableChooser<>();
    public static SendableChooser<Integer> HeightGrid = new SendableChooser<>();
    public static SendableChooser<Integer> PositionGrid = new SendableChooser<>();
    public static SendableChooser<Integer> ConeVCube = new SendableChooser<>();
    public final int gridTag1  = 0;
    public final int gridTag2 = 1;
    public final int gridTag3 = 2;
    public final int gridHeightLow = 0;
    public final int gridHeightMid = 1;
    public final int gridHeightHigh = 2;
    public final int gridPosLeft = 0;
    public final int gridPosMid = 1;
    public final int gridPosRight =2;
    
    public int tag;
    public int hor;
    public int height;

    public double gyroArtificialOffset;

   


  

private  SwerveDrivePoseEstimator m_poseEstimator;
    public Swerve() {
       

    
        pcw = new PhotonCameraWrapper();
        gyro  = new AHRS();
        
        
        // zeroGyro();
        SmartDashboard.putData("Field", m_fieldSim);
        aimPID = new PIDController(0.05, 0, 0);
        aimPID.setTolerance(0.5);

        rotatePID = new ProfiledPIDController(
            0.010,
            0,
            0.001, new TrapezoidProfile.Constraints(1, 3));

    rotatePID.setTolerance(0.001);
 
        mSwerveModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, m_frontRight.getCanCoder()),
            new SwerveModulePosition(0, m_frontLeft.getCanCoder()),
            new SwerveModulePosition(0, m_backRight.getCanCoder()),
            new SwerveModulePosition(0, m_backLeft.getCanCoder())
          };


        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), mSwerveModulePositions);
 
        m_poseEstimator = new SwerveDrivePoseEstimator( Constants.Swerve.swerveKinematics, getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()}, getPose());
    }

    public void kyslol(){
        tag = AprilTagGrid.getSelected();
        hor = PositionGrid.getSelected();
        height = HeightGrid.getSelected();

        GlobalVariables.height = HeightGrid.getSelected();


        // gamePiece = ConeVCube.getSelected();
    }

    
    public void updateOdometry() {
               
        // Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());
      
        m_poseEstimator.update(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()});
        // if (result.isPresent()) {

            // EstimatedRobotPose camPose = result.get();

            // m_poseEstimator.addVisionMeasurement(
            //         camPose.estimatedPose.toPose2d(), camPose.timestampSeconds); 
                
            // m_fieldSim.getObject("Cam Est Pos").setPose(  m_poseEstimator.getEstimatedPosition() );
        // } else {
        //     // m_fieldSim.getObject("Cam Est Pos").setPose(m_poseEstimator.getEstimatedPosition());
        // }

         m_poseEstimator.update(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()});

        m_fieldSim.setRobotPose(getPose());
        SmartDashboard.putNumber("lmao X", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("lmao Y",  m_poseEstimator.getEstimatedPosition().getY());
        // SmartDashboard.putBoolean("AprilTag in View", result.isPresent());
        // m_fieldSim.setRobotPose(getPose());
        // m_fieldSim.getObject("Actual Pos").setPose(getPose());
        // m_fieldSim.setRobotPose(m_poseEstimator.getEstimatedPosition());
    
    }

    public void updateRobotDirection(){
        if (DriverStation.getAlliance() == Alliance.Blue){
            if(gyro.getYaw()>=90 || gyro.getYaw() <= -90){
                GlobalVariables.robotDirection = true;
            }
            else{
                GlobalVariables.robotDirection = false;
            }
        }
        else{
            if(gyro.getYaw()>=90 || gyro.getYaw() <= -90){
                GlobalVariables.robotDirection = false;
            }
            else{
                GlobalVariables.robotDirection = true;
            } 
        }
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

    public Pose2d getGlobalPose(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        // swerveOdometry.resetPosition(getYaw(), mSwerveModulePositions, pose);
        swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
          }, pose);
    }

    public void normalizeOdometry(){
        swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()}, m_poseEstimator.getEstimatedPosition());
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
        
        gyroArtificialOffset = 0;
        // gyro.get
    }

    public void autoZeroGyro(){
        gyro.zeroYaw();
        // gyroArtificialOffset = yaw;
        gyroArtificialOffset =  getAprilTagEstPosition().getRotation().getDegrees() ;
    }
    
    
    public Rotation2d getYaw() {
        // if (gyro.isMagnetometerCalibrated()) {
            // return Rotation2d.fromDegrees( gyro.getYaw() );
        //   }
           return Rotation2d.fromDegrees(360.0 - (gyro.getYaw() + gyroArtificialOffset));
    }

    public double getPitch(){
        return gyro.getRoll();
    }

    public double getCamOffset(){
        pcw.cameraPipelines(GlobalVariables.gamePiece);
        double headingError;
        if(GlobalVariables.robotDirection){
            headingError = pcw.getFrontCamOffset();
            return aimPID.calculate(headingError, 30);
        }
        else{
            headingError = pcw.getRearCamOffset();
            return aimPID.calculate(headingError, 9);

        }


    }

        
    // public double frontCamOffset(int pipeline){
    //     pcw.frontCamPipeline(GlobalVariables.gamePiece);
        
    //     double headingError = pcw.getYOffset();
    //    /*  double steering_adjust;
    //     steering_adjust = 0;
    //     if (headingError > 0.05)
    //     {
    //             steering_adjust =  0.09;
    //     }
    //     else if (headingError < -0.05)
    //     {
    //             steering_adjust = -0.09;
    //     } */


    //     return (rotatePID.calculate(headingError, 0)/* +steering_adjust */);
    // }    

           
    // public double AutoFrontCamOffset(int pipeline){
    //     pcw.frontCamPipeline(GlobalVariables.gamePiece);
        
    //     double headingError = pcw.AutoGetYOffset();
    //    /*  double steering_adjust;
    //     steering_adjust = 0;
    //     if (headingError > 0.05)
    //     {
    //             steering_adjust =  0.09;
    //     }
    //     else if (headingError < -0.05)
    //     {
    //             steering_adjust = -0.09;
    //     } */


    //     return (rotatePID.calculate(headingError, 0)/* +steering_adjust */);
    // }    

    




    @Override
    public void periodic(){
        updateRobotDirection();
        // pcw.frontCamPipeline(2);
        pcw.cameraPipelines(GlobalVariables.gamePiece);

        AprilTagGrid.setDefaultOption("Left Tag", gridTag1);
        AprilTagGrid.addOption("Mid Tag", gridTag2);
        AprilTagGrid.addOption("Right Tag", gridTag3);
    
        HeightGrid.setDefaultOption("Low Height", gridHeightLow);
        HeightGrid.addOption("Mid Height", gridHeightMid);
        HeightGrid.addOption("High Height", gridHeightHigh);
    
        PositionGrid.setDefaultOption("Left Position", gridPosLeft);
        PositionGrid.addOption("Mid Position", gridPosMid);
        PositionGrid.addOption("Right Position", gridPosRight);

     
        SmartDashboard.putData(AprilTagGrid);
        SmartDashboard.putData(HeightGrid);
        SmartDashboard.putData(PositionGrid);


        SmartDashboard.putNumber("Odometry X", getPose().getX());
        SmartDashboard.putNumber("Odometry Y", getPose().getY());
        // SmartDashboard.putNumber("CONE OFFSET ", frontCamOffset(1));



    SmartDashboard.putNumber("Gyro Yaw ", gyro.getYaw());
    SmartDashboard.putNumber("AprilTag Yaw ",  getAprilTagEstPosition().getRotation().getDegrees());
    SmartDashboard.putBoolean("robot direction ", GlobalVariables.robotDirection);
    SmartDashboard.putNumber("Rear Cam offset ", pcw.getRearCamOffset());

        kyslol();

         swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backRight.getPosition(),
            m_backLeft.getPosition()
          });  

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}