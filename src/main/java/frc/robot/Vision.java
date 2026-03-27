package frc.robot;

import java.util.Vector;
import java.util.function.DoubleBinaryOperator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;



import static edu.wpi.first.units.Units.*;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {

    String k_limelightName = "limelight"; //default name of the limelight, can be changed in the limelight settings   

    final Distance k_xOffset = Inch.of(-27.25/2+6); //front/back offest of camera to robot center (positive is forward, negative is backward)
    final Distance k_yOffset = Inch.of((27.25/2-3.125)); //left/right offset of camera to robot center (positive is left, negative is right)
    final Distance k_zOffset = Inch.of(22-0.75); //vertical offset of camera to robot center (positive is up, negative is down)
    final Angle k_rollOffset = Degree.of(0.0); //roll offset of camera to robot center (positive is clockwise, negative is counterclockwise)    
    final Angle k_pitchOffset = Degree.of(27.29); //pitch offset of camera to robot center (positive is up, negative is down) - 12.4 original 
    final Angle k_yawOffset = Degree.of(0.0); //yaw offset of camera to robot center (positive is left, negative is right)

    private CommandSwerveDrivetrain m_poseEstimator;

    // double x=LimelightHelpers.getTX(null); //horizontal offset
    // double y=LimelightHelpers.getTY(null); //vertical offset
    // double a=LimelightHelpers.getTA(null); //target area (ranges from 0%-100%)
    // boolean hasTarget = LimelightHelpers.getTV(null); //makes sure there is a vaild target present

    // double txnc = LimelightHelpers.getTXNC(null); //horizontal offset from principal pixel/point to target in degrees
    // double tync = LimelightHelpers.getTYNC(null); //vertical offset from principal pixel/point to target in degrees

    public Vision(CommandSwerveDrivetrain poseEstimator)
    {
        m_poseEstimator = poseEstimator;

        LimelightHelpers.setPipelineIndex(k_limelightName, 0);

        LimelightHelpers.setCameraPose_RobotSpace(k_limelightName, 
        k_xOffset.in(Meter), 
        k_yOffset.in(Meter), 
        k_zOffset.in(Meter), 
        k_rollOffset.in(Degree), 
        k_pitchOffset.in(Degree), 
        k_yawOffset.in(Degree));





    }

    double lastTimestamp = 0.0;  
    public void megaTagPose_periodic()
    {
        Pose2d currentPose = m_poseEstimator.samplePoseNow();
        // System.out.println(currentPose);
        LimelightHelpers.SetRobotOrientation(k_limelightName,
            currentPose.getRotation().getDegrees() , 
            0, 
            0, 
            0,
             0, 
             0
             );

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(k_limelightName);
        boolean reject_update = false;


        // if (Math.abs(m_poseEstimator.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())>360){ //if the robot is rotating faster than 360 degrees per second, ignore vision measurements to prevent pose estimator from diverging
        
        //     reject_update = true;
        // }

        if (mt2 != null) {
            // System.out.println("Cameraing");
            
        

            lastTimestamp = mt2.timestampSeconds;
            
            if(mt2.tagCount == 0) {
                reject_update = true;
            }
            if (!reject_update)
            {
                
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
                m_poseEstimator.addVisionMeasurement(mt2.pose, lastTimestamp);
            }

        }
        else {
            lastTimestamp = lastTimestamp+0.02; //if no target is detected, increment the timestamp to prevent the pose estimator from rejecting future measurements due to old timestamps
        }

        

    }
}


