// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
/** Add your docs here. */
public class PhotonCameraWrapper 
{
    public PhotonCamera camera;
    public RobotPoseEstimator robotPoseEstimator;

    public PhotonCameraWrapper() {
        final AprilTag tag02 = 
                new AprilTag(01, new Pose3d(
                                                new Pose2d(
                                                    0,
                                                    Units.feetToMeters(3 + 1.5/12) ,
                                                    Rotation2d.fromDegrees(0.0))));

        final AprilTag tag01 =
                new AprilTag(02,
                                    new Pose3d(
                                                new Pose2d(
                                                   0,
                                                   Units.feetToMeters(8 + 11/12), 
                                                    Rotation2d.fromDegrees(0.0))));
         final AprilTag tag03 =
                 new AprilTag(03,
                                     new Pose3d(
                                                 new Pose2d(
                                                    0,
                                                    Units.feetToMeters(14 + 8.5/12), 
                                                     Rotation2d.fromDegrees(0.0))));
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        
        atList.add(tag02);
        atList.add(tag01);
         atList.add(tag03);
        AprilTagFieldLayout atf1 = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);
        camera = new PhotonCamera(VisionConstants.cameraName);

        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.robotToCam));
        
        robotPoseEstimator = new RobotPoseEstimator(atf1, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    // public double horOffset(){
    //            var result = camera.getLatestResult();

    //     return result.getBestTarget().getYaw();
    // }

    // public boolean hasTarget(){
    //     var result = camera.getLatestResult();

    //     return result.hasTargets();
    // }

    
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent() && result.get().getFirst() != null) {
            return new Pair<Pose2d, Double>( result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        } else {
            return new Pair<Pose2d, Double>(prevEstimatedRobotPose, 0.0);
        }
    }
}
