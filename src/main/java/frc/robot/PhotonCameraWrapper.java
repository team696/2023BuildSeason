// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;


import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
/** Add your docs here. */
public class PhotonCameraWrapper 
{
    public PhotonCamera camera1;
    public PhotonCamera camera2;
    public PhotonPoseEstimator photonPoseEstimator;
    public PhotonPoseEstimator photonPoseEstimator2;
    

    public PhotonCameraWrapper() {
        AprilTagFieldLayout aprilTagFieldLayout;
        try {
         aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
         camera1 = new PhotonCamera(VisionConstants.camera1Name);
         camera2 = new PhotonCamera(VisionConstants.camera2Name);

         photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera2, VisionConstants.robotToCam1);
         photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera1, VisionConstants.robotToCam2);
            
        } catch (Exception e) {
            System.out.println("can't find field layout april tag.");
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            if (photonPoseEstimator == null || photonPoseEstimator2 == null) 
                return Optional.empty();

            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
            Optional<EstimatedRobotPose> ppe1 = photonPoseEstimator.update();
            Optional<EstimatedRobotPose> ppe2 = photonPoseEstimator2.update();
            if (ppe2.isPresent()) {
                
                if (ppe1.isPresent())  {
                    photonPoseEstimator2.setReferencePose(ppe1.get().estimatedPose);
                } 
                return photonPoseEstimator2.update();
            } 
            else {
                return photonPoseEstimator.update();
            }
    }
}
