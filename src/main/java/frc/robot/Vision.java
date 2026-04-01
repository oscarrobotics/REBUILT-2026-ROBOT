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
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.*;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision {

    String k_limelightName = "limelight"; //default name of the limelight, can be changed in the limelight settings   
    String k_limelightfollowerName = "followerlimelight"; //2nd limelight 

    

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
        Inch.of(-13).in(Meter), 
        Inch.of(0).in(Meter), 
        Inch.of(22-4).in(Meter), 
        Degree.of(0.50).in(Degree), 
        Degree.of(27.150).in(Degree), 
        Degree.of(180.0).in(Degree));

        LimelightHelpers.setCameraPose_RobotSpace(k_limelightfollowerName, 
        Inch.of(-12).in(Meter), 
        Inch.of(-12).in(Meter), 
        Inch.of(22-2).in(Meter), 
        Degree.of(0.0).in(Degree), 
        Degree.of(12).in(Degree), 
        Degree.of(0).in(Degree));



        //limelight configuration 
        // LimelightHelpers.SetIMUAsssitAlpha(k_limelightName, 0.001);
        // LimelightHelpers.SetIMUAsssitAlpha(k_limelightfollowerName, 0.001);
        


    }

    double lastTimestamp = 0.0;  
    public void megaTagPose_periodic()
    {


        // update gyro seed if disabled 
        if (DriverStation.isDisabled()){
            LimelightHelpers.SetIMUMode(k_limelightName, 1);
            LimelightHelpers.SetIMUMode(k_limelightfollowerName, 1);
        }


        //set gyro to be internal if running 
        if (!DriverStation.isDisabled()){
            LimelightHelpers.SetIMUMode(k_limelightName, 4);
            LimelightHelpers.SetIMUMode(k_limelightfollowerName, 4);
        }



        //set update behaviour depending on teleop or auton




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
        
        LimelightHelpers.SetRobotOrientation(k_limelightfollowerName, 
        currentPose.getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0
        );

        LimelightHelpers.PoseEstimate shooter_result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(k_limelightName);
        LimelightHelpers.PoseEstimate forward_result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(k_limelightfollowerName);
        //  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(k_limelightName);
        boolean reject_update = false;


        if (Math.abs(m_poseEstimator.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())>360){ //if the robot is rotating faster than 360 degrees per second, ignore vision measurements to prevent pose estimator from diverging
        
            reject_update = true;
        }

        if (shooter_result != null) {
            // System.out.println("Cameraing");
            
        

            lastTimestamp = shooter_result.timestampSeconds;
            
            if(shooter_result.tagCount < 1) {
                reject_update = true;
            }
            // if(mt2.tagCount<2 && Drive)
            if (!reject_update)
            {
                
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
                m_poseEstimator.addVisionMeasurement(shooter_result.pose, lastTimestamp);
            }

        }
        else {
            lastTimestamp = lastTimestamp+0.02; //if no target is detected, increment the timestamp to prevent the pose estimator from rejecting future measurements due to old timestamps
        }

        

    }
}


