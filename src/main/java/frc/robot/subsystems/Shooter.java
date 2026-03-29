package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;


public class Shooter extends SubsystemBase{
    
    private final CommandSwerveDrivetrain m_poseEstimator;
   //identifying motors + encoders with their ids 
    private final SparkFlex m_shooterleader = new SparkFlex(7, MotorType.kBrushless); //RIGHT (robot facing front)
    private final SparkFlex m_shooterfollower = new SparkFlex(6, MotorType.kBrushless); //LEFT SIDE (robot facing front)
    
    private final RelativeEncoder m_shooterleader_encoder = m_shooterleader.getEncoder();
    private final RelativeEncoder m_shooterfollower_encoder = m_shooterfollower.getEncoder();
    
    // Initialize the closed loop controller -> using closed loop control system for shooter mechanism 
     private SparkClosedLoopController m_controllerleader;

    

    // max RPM 
     public AngularVelocity k_maxShooterRPM; 
    
    // current target RPM 
     private AngularVelocity m_targetRPM = RPM.of(0);

    //  private final AngularVelocity m_default_speed = RPM.of(2800);


     public Distance min_distance = Meters.of(1.7);
     public Distance opt_distance = Meters.of (3.1);
     public Distance max_distance = Meters.of(5.0);
    
     public  AngularVelocity min_speed = RPM.of(2800/1.03);
     public  AngularVelocity opt_speed = RPM.of(3200/1.03);
     public  AngularVelocity max_speed = RPM.of(5200);

     public AngularVelocity speed_setpoint = opt_speed;

     public AngularVelocity close_speed = RPM.of(2500);
     public AngularVelocity far_speed = RPM.of(5000);
     public AngularVelocity full_speed = RPM.of(6500);

     
   
    
     //shuffleboard tab entries 
     private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
     private final GenericEntry kPEntry = tab.add("kP", 0.01).getEntry();
     private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
     private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
     private final GenericEntry maxRPMEntry = tab.add("Max RPM", 6784).getEntry();
     private final GenericEntry minOutputEntry = tab.add("Min Output", -1.0).getEntry();
     private final GenericEntry maxOutputEntry = tab.add("Max output", 1.0).getEntry();
     private final GenericEntry leaderRPMEntry = tab.add("Leader RPM", 0.0).getEntry();
     private final GenericEntry followerRPMEntry = tab.add("Follower RPM", 0.0).getEntry();
     private final GenericEntry targetRPMEntry = tab.add("Target RPM", 0.0).getEntry();
     private final GenericEntry targetDistanceEntry = tab.add("Target Distance (m)", 0.0).getEntry();
     private final GenericEntry targetSpeedEntry = tab.add("Target Speed (RPM)", 0.0).getEntry();
     private final GenericEntry angleToTargetEntry = tab.add("Angle to Target (degrees)", 0.0).getEntry();
     private final GenericEntry angleDelta = tab.add("Angle Delta (degrees)", 0.0).getEntry();
     
     private final GenericEntry motorvoltageEntry = tab.add("Motor Voltage", 0.0).getEntry();
     private final GenericEntry motorCurrentEntry = tab.add("Motor Current", 0.0).getEntry();
     private final GenericEntry motorTemperatureEntry = tab.add(" Leader Motor Temperature (C)", 0.0).getEntry();
     private final GenericEntry followerMotorTemperatureEntry = tab.add("Follower Motor Temperature (C)", 0.0).getEntry();
     
     
     


    public Shooter(CommandSwerveDrivetrain poseEstimator)
    {
        m_poseEstimator = poseEstimator;
        configureMotors();

          //initializing max RPM 
        k_maxShooterRPM = RPM.of(maxRPMEntry.getDouble(6784));
   
    }

    //creating the configuration process which will set limits for shooting + adjustments/tuning for velocity 
                
    private void configureMotors(){
      
      //for leader motor 

      m_controllerleader = m_shooterleader.getClosedLoopController();

      SparkFlexConfig shooterleaderConfig = new SparkFlexConfig();
        shooterleaderConfig.idleMode(IdleMode.kCoast);
        shooterleaderConfig.smartCurrentLimit(80);
        shooterleaderConfig.secondaryCurrentLimit(120);

        shooterleaderConfig.closedLoop
                // .p(kPEntry.getDouble(0.00001))
                // .i(kIEntry.getDouble(0.0))
                // .d(kDEntry.getDouble(0.0))
                .p(0.00001)
                .i(0.0)
                .d(0.0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(minOutputEntry.getDouble(-1.0), maxOutputEntry.getDouble(1.0))
                .feedForward.kV(12.0/6784.0*1.03);
               
                


          
                
                //shooterleaderConfig.closedLoop.feedForward.kV(12/5767);
                
        m_shooterleader.configure(shooterleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
      //for follower motor 
        SparkFlexConfig shooterfollowerConfig = new SparkFlexConfig();
        shooterfollowerConfig.idleMode(IdleMode.kCoast);
        shooterfollowerConfig.smartCurrentLimit(80);
        shooterfollowerConfig.follow(m_shooterleader, true); // follows leader motor
       
      
        m_shooterfollower.configure(shooterfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        

    }
       //Shooter controls 

        //method to run the shooter 
        public void StartShooter(AngularVelocity velocity){
          m_targetRPM = velocity;
          

          if (velocity.lt(k_maxShooterRPM)) {
            m_controllerleader.setSetpoint(
            velocity.in(RPM),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0          
            );
          }
          else{
             m_controllerleader.setSetpoint(
            opt_speed.in(RPM),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0          
            );
            System.out.println("no target:opt velocity used");
          

          }
          // System.out.println("shooting");
          // System.out.println(k_maxShooterRPM);
          // System.out.println(velocity);
          // System.out.print("setpoint: ");
          // System.out.println(m_controllerleader.getSetpoint());
        }

        // public void shoot_with_set_speed( CommandXboxController opStick){

          



        // }

        //method to stop the shooter 
        public void StopShooter() {
          
          m_targetRPM = RPM.of(0);
          m_controllerleader.setSetpoint(
            0, ControlType.kVoltage,
            ClosedLoopSlot.kSlot0
            );
        }

        

        //command to stop shooter 
        public Command stopCommand() {
                  return run(this::StopShooter);
          
        }
        //live shuffleboard upates 
        // public void updatePID(){
        //     double newP = kPEntry.getDouble(0.01);
        //     double newI = kIEntry.getDouble(0.0);
        //     double newD = kDEntry.getDouble(0.0);
        //     double newMin = minOutputEntry.getDouble(-1.0);
        //     double newMax = maxOutputEntry.getDouble(1.0);
        
        //     SparkFlexConfig config = new SparkFlexConfig();
        //     config.idleMode(IdleMode.kCoast);
        //     config.smartCurrentLimit(80);
        //     config.closedLoop.p(newP).i(newI).d(newD).outputRange(newMin, newMax);

        //     m_shooterleader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //     k_maxShooterRPM = RPM.of(maxRPMEntry.getDouble(5767));
        // }
      
        public boolean shooteratSpeed(){
          double currentRPM = m_shooterleader_encoder.getVelocity();
          double tolerance = 100;
          return m_targetRPM.isNear(RPM.of(currentRPM), RPM.of(tolerance));
        }

        public boolean shooterAimed(){
          
          return m_poseEstimator.get_target_angle_differnce().isNear(Degree.of(0),Degree.of(5) );
          //  || m_poseEstimator.get_target_angle_differnce().isNear(Degree.of(0),Degree.of(5) );

        }


        public AngularVelocity distance2speed(Distance target_distance){

            AngularVelocity target_speed ;
            
            if (target_distance.lt(opt_distance)){
              target_speed = opt_speed.minus(min_speed).times(target_distance.minus(min_distance).div(opt_distance.minus(min_distance))).plus(min_speed);
            }
            else {
              target_speed = max_speed.minus(opt_speed).times(target_distance.minus(opt_distance).div(max_distance.minus(opt_distance))).plus(opt_speed);
            }
            targetSpeedEntry.setDouble(target_speed.in(RPM));
            return target_speed;
        }

       

        public AngularVelocity get_auto_speed(){

          return distance2speed(m_poseEstimator.get_target_distance().minus(Meters.of(0.3)));
        }

        public AngularVelocity get_moving_auto_speed(){
          return distance2speed(m_poseEstimator.get_target_moving_distance());
        }

        public AngularVelocity get_target_speed(){
          
          return RPM.of(2800);
        }

        //auto to shoot preloaded fuel 
        public Command autoshoot(){
          
          return runOnce(() -> {StartShooter(get_auto_speed());})
          .andThen(new WaitCommand(0.4));
          // .until(this::shooteratSpeed)

            // .withTimeout(2);

            
        }
        
      
        
          
        

        //Shuffleboard Updates */ 
        @Override
        public void periodic() {
          
        //  updatePID();

        leaderRPMEntry.setDouble(m_shooterleader_encoder.getVelocity());
        followerRPMEntry.setDouble(m_shooterfollower_encoder.getVelocity());
        targetRPMEntry.setDouble(m_targetRPM.in(RPM));
        targetDistanceEntry.setDouble(m_poseEstimator.get_target_distance().in(Meter));
        // targetSpeedEntry.setDouble(get_target_speed().in(RPM));
        angleToTargetEntry.setDouble(m_poseEstimator.get_target_angle().in(Degree));
        angleDelta.setDouble(m_poseEstimator.get_target_angle_differnce().in(Degree));
        motorvoltageEntry.setDouble(m_shooterleader.getAppliedOutput()*12);
        motorCurrentEntry.setDouble(m_shooterleader.getOutputCurrent());

        motorTemperatureEntry.setDouble(m_shooterleader.getMotorTemperature());
        followerMotorTemperatureEntry.setDouble(m_shooterfollower.getMotorTemperature());


     }
  }       
                       
            