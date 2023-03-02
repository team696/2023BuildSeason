// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
/** Add your docs here. */
public class PhotonCameraWrapper 
{
    public PhotonCamera camera1;
    public PhotonCamera camera2;
    public PhotonCamera frontCam;
    public PhotonPoseEstimator photonPoseEstimator;
    public PhotonPoseEstimator photonPoseEstimator2;
    

    public PhotonCameraWrapper() {
        AprilTagFieldLayout aprilTagFieldLayout;
        try {
         aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
         camera1 = new PhotonCamera(VisionConstants.camera1Name);
         camera2 = new PhotonCamera(VisionConstants.camera2Name);

         frontCam = new PhotonCamera(VisionConstants.frontCamName);
         photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera2, VisionConstants.robotToCam1);
         photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera1, VisionConstants.robotToCam2);
            
        } catch (Exception e) {
            System.out.println(e);
        }
    }
 public void frontCamPipeline(int pipelineIndex){
        frontCam.setPipelineIndex(pipelineIndex);;
    }
    public double getYOffset(){
        var result = frontCam.getLatestResult();

        if (!result.hasTargets())
            return 0;
      
      
            return  result.getBestTarget().getYaw();
        
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
