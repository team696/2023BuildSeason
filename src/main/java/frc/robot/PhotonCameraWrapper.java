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
    public PhotonCamera frontCam;
    public PhotonCamera rearCam;
    public PhotonPoseEstimator photonPoseEstimator;
    public PhotonPoseEstimator photonPoseEstimator2;
    

    public PhotonCameraWrapper() {
        AprilTagFieldLayout aprilTagFieldLayout;
        try {
         aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
         frontCam = new PhotonCamera(VisionConstants.camera1Name);
         rearCam = new PhotonCamera(VisionConstants.camera2Name);

        //  photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, frontCam, VisionConstants.robotToCam1);
        //  photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, org.photonvision.PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, rearCam, VisionConstants.robotToCam2);
            
        } catch (Exception e) {
            System.out.println(e);
        }
    }
//  public void frontCamPipeline(int pipelineIndex){
//         frontCam.setPipelineIndex(pipelineIndex);;
//     }

public void cameraPipelines(int pipeline){
    frontCam.setPipelineIndex(pipeline);
    rearCam.setPipelineIndex(pipeline);
}

public double getFrontCamOffset(){
    var result = frontCam.getLatestResult();

    if (!result.hasTargets())
        return 9;
  
  
        return  result.getBestTarget().getYaw();
    
}

public double getRearCamOffset(){
    var result = frontCam.getLatestResult();

    if (!result.hasTargets()){
        return 9;
    }
    else{
  
  
        return  result.getBestTarget().getYaw();
    }
}

public double getFrontCamDistance(){
    var result = frontCam.getLatestResult();

    if (!result.hasTargets())
        return 0;
  
  
        return  result.getBestTarget().getBestCameraToTarget().getX();
    
}

public double getRearCamDistance(){
    var result = frontCam.getLatestResult();

    if (!result.hasTargets())
        return 0;
  
  
        return  result.getBestTarget().getBestCameraToTarget().getX();
    
}







    // public double getYOffset(){
    //     var result = frontCam.getLatestResult();

    //     if (!result.hasTargets())
    //         return 0;
      
      
    //         return  result.getBestTarget().getYaw();
        
    // }

    // public double AutoGetYOffset(){
    //     var result = frontCam.getLatestResult();

    //     if (!result.hasTargets())
    //         return 0.5;
      
      
    //         return  result.getBestTarget().getYaw();
        
    // }
    
    // public double AutoTurnTOCOne(){
    //     var result = frontCam.getLatestResult();

    //     if (!result.hasTargets())
    //         return 0.5;
      
      
    //         return  result.getBestTarget().getYaw();
        
    // }

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
