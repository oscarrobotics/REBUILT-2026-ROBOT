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

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.*;

import static edu.wpi.first.units.Units.*;

import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

 import edu.wpi.first.math.MathUtil;

public class Vision {

    String k_limelightName = "limelight"; //default name of the limelight, can be changed in the limelight settings   
    String k_limelightfollowerName = "limelight-follow"; //2nd limelight 

    

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
        Inch.of(-13).in(Meter),                 // X
        Inch.of(0).in(Meter),         // Y
        Inch.of(22-4).in(Meter),                // Z
        Degree.of(0.50).in(Degree),   // Roll
        Degree.of(27.150).in(Degree), // Pitch
        Degree.of(180.0).in(Degree)); // Yaw
        
        LimelightHelpers.setPipelineIndex(k_limelightfollowerName, 0);
        
        LimelightHelpers.setCameraPose_RobotSpace(k_limelightfollowerName, 
        Inch.of(-12).in(Meter),             //x
        Inch.of(-12).in(Meter),             //y
        Inch.of(22-3).in(Meter),            //z
        Degree.of(5).in(Degree),  //roll
        Degree.of(9).in(Degree),  //pitch
        Degree.of(0).in(Degree)); //yaw


        //limelight configuration 
        // LimelightHelpers.SetIMUAsssitAlpha(k_limelightName, 0.001);
        // LimelightHelpers.SetIMUAsssitAlpha(k_limelightfollowerName, 0.001);
        


    }
/*An IMU, or Inertial Measurement Unit, is an electronic sensor 
used in FIRST Robotics Competition (FRC) to track a robot's orientation, 
rotational speed, and linear acceleration. */
    double lastTimestamp = 0.0;  
    public void megaTagPose_periodic() //This is the actual field loaclization function
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




        Pose2d currentPose = m_poseEstimator.samplePoseNow(); //give the robots current pose
        // System.out.println(currentPose);
        //gives the gyro info to limelight. It does this because of how MegaTag2 works. 
        //MegaTag2 gives the robot gyro information to optimize Limelight robot localization
        LimelightHelpers.SetRobotOrientation(k_limelightName,
            currentPose.getRotation().getDegrees(),0,0,0,0,0
             );
        //give the gyro info to the linelight.
        LimelightHelpers.SetRobotOrientation(k_limelightfollowerName, 
            currentPose.getRotation().getDegrees(),0,0,0,0,0
            );

            //Limelights use the given data to predict the pose of the robot.
        LimelightHelpers.PoseEstimate shooter_result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(k_limelightName);
        LimelightHelpers.PoseEstimate forward_result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(k_limelightfollowerName);
        //  LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(k_limelightName);
        
        //filters the results from the calculation above
        process_mt_result(shooter_result);
        process_mt_result(forward_result);
        


    }

    public void process_mt_result(PoseEstimate poseEstimate){
        boolean reject_update = false;

//if the robot is rotating faster than 360 degrees per second, 
//ignore vision measurements to prevent pose estimator from diverging
//gets angular velovity from pigeon on robot
        if (Math.abs(m_poseEstimator.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())>360){ 
        
            reject_update = true;
        }

        if (poseEstimate != null) {
            // System.out.println("Cameraing");
            
        
//----------------------------------------------------
            
            //first: if the robot sees no tags (x<1), reject the measurement
            //second: If the robot is in autonomous AND the Limelight sees 
            //fewer than 2 tags, reject the measurement.
            if(poseEstimate.tagCount < 1 || (poseEstimate.tagCount < 2 && DriverStation.isAutonomous()))
              {
                reject_update = true;
            }
            // if(mt2.tagCount<2 && Drive)
            
            //"If reject_update is NOT true."
            if (!reject_update)
            {  
                // LimelightHelpers.get
                
                //sets trust in limelight measurements/data.
                //X and Y have a 0.8 degree of uncertainty
                //Rotations has 9999999 of uncertainty,
                //this tells the robot to not trust our rotations at all if 
                //we see no tags in tele-op, or we see fewer than 2 tags in auto
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.8,0.8,9999999));
                m_poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
            }

        }
        


    }

    public void seedFieldCentricFromCamera(){
//m_poseEstimator.samplePoseNow gets robots current pose
//seedFieldCentric() establishes what direction the robot 
//considers "forward" for field-centric driving.
//not quite sure why we have a backup(?) field centric thingy with the cameras (?)
        m_poseEstimator.seedFieldCentric(new Rotation2d(m_poseEstimator.samplePoseNow().getRotation().getDegrees()));
    }
      
    
  
//I will sacrifice two new-born children if you work

// ---------- Distance (Limelight "Estimating Distance" math) ----------
private static final double kMountAngleDeg    = 27.15; // camera tilt from vertical
private static final double kLensHeightInches = 18.0;  // lens center to floor
private static final double kGoalHeightInches = 44.0;  // PLACEHOLDER: floor to center of the tag you aim at

// ---------- Range / aim control ----------
private static final double kDesiredDistanceInches = 100.0; // TODO: your shooting distance
private static final double kToleranceInches = 3.0;
private static final double kPDistance = 0.03;  // m/s per inch of error
private static final double kMaxSpeed  = 1.5;   // m/s
private static final double kPAim      = 0.05;  // rad/s per degree of tx
private static final double kMaxAimRate = 2.0;  // rad/s
// +1 if the camera faces the robot's front, -1 if it faces the rear (your commented pose has yaw 180, so probably -1)
private static final double kCameraFacingSign = -1.0;

public boolean hasGoalTarget() {
    return LimelightHelpers.getTV(k_limelightName);
}

/** Inches from camera to goal, or -1 if no target. */
public double getDistanceToGoalInches() {
    if (!hasGoalTarget()) return -1;
    double ty = LimelightHelpers.getTY(k_limelightName);
    double angleRad = Math.toRadians(kMountAngleDeg + ty);
    return (kGoalHeightInches - kLensHeightInches) / Math.tan(angleRad);
}

/** Robot-relative forward velocity (m/s). Positive error = too far from the goal. */
public double getRangeVelocityX() {
    double d = getDistanceToGoalInches();
    if (d < 0) return 0;
    double error = d - kDesiredDistanceInches;
    if (Math.abs(error) < kToleranceInches) return 0;
    return MathUtil.clamp(kCameraFacingSign * kPDistance * error, -kMaxSpeed, kMaxSpeed);
}

/** Rotation rate (rad/s) that centers the target in the camera. */
public double getAimRotationRate() {
    if (!hasGoalTarget()) return 0;
    double tx = LimelightHelpers.getTX(k_limelightName);
    if (Math.abs(tx) < 1.0) return 0;
    return MathUtil.clamp(-kPAim * tx, -kMaxAimRate, kMaxAimRate);
}

public boolean inRange() {
    double d = getDistanceToGoalInches();
    return d >= 0 && Math.abs(d - kDesiredDistanceInches) < kToleranceInches;
}
    
}


