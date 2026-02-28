package frc.robot;


public class Vision {
double x=LimelightHelpers.getTX(null); //horizontal offset
double y=LimelightHelpers.getTY(null); //vertical offset
double a=LimelightHelpers.getTA(null); //target area (ranges from 0%-100%)
boolean hasTarget = LimelightHelpers.getTV(null); //makes sure there is a vaild target present

double txnc = LimelightHelpers.getTXNC(null); //horizontal offset from principal pixel/point to target in degrees
double tync = LimelightHelpers.getTYNC(null); //vertical offset from principal pixel/point to target in degrees

public Vision()
{
    LimelightHelpers.setPipelineIndex(null, 0);

}

public static void megaTagPose_periodic() {
   
}


}


