package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;


public class Shooter extends SubsystemBase{
    
   //identifying motors + encoders with their ids 
    private final SparkFlex m_shooterleader = new SparkFlex(6, MotorType.kBrushless); //RIGHT (robot facing front)
    private final SparkFlex m_shooterfollower = new SparkFlex(7, MotorType.kBrushless); //LEFT SIDE (robot facing front)
    
    private final RelativeEncoder m_shooterleader_encoder = m_shooterleader.getEncoder();
    private final RelativeEncoder m_shooterfollower_encoder = m_shooterfollower.getEncoder();
    
    // Initialize the closed loop controller -> using closed loop control system for shooter mechanism 
     private SparkClosedLoopController m_controllerleader;

    // max RPM 
     public AngularVelocity k_maxShooterRPM; 
    
    // current target RPM 
     private double m_targetRPM = 0;
   
    
     //shuffleboard tab entries 
     private final ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
     private final GenericEntry kPEntry = tab.add("kP", 0.01).getEntry();
     private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
     private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
     private final GenericEntry kFFEntry = tab.add("kFF", 12.0 / 3000.0).getEntry();
     private final GenericEntry maxRPMEntry = tab.add("Max RPM", 5767).getEntry();
     private final GenericEntry minOutputEntry = tab.add("Min Output", -1.0).getEntry();
     private final GenericEntry maxOutputEntry = tab.add("Max output", 1.0).getEntry();
     private final GenericEntry leaderRPMEntry = tab.add("Leader RPM", 0).getEntry();
     private final GenericEntry followerRPMEntry = tab.add("Follower RPM", 0).getEntry();


    public Shooter(){
        configureMotors();

          //initializing max RPM 
        k_maxShooterRPM = RPM.of(maxRPMEntry.getDouble(5767));
   
    }

    //creating the configuration process which will set limits for shooting + adjustments/tuning for velocity 
                
    private void configureMotors(){
      
      //for leader motor 

       m_controllerleader = m_shooterleader.getClosedLoopController();

      SparkFlexConfig shooterleaderConfig = new SparkFlexConfig();
        shooterleaderConfig.idleMode(IdleMode.kCoast);
        shooterleaderConfig.smartCurrentLimit(40);

        shooterleaderConfig.closedLoop
                .p(kPEntry.getDouble(0.01))
                .i(kIEntry.getDouble(0.0))
                .d(kDEntry.getDouble(0.0))
                .outputRange(minOutputEntry.getDouble(-1.0), maxOutputEntry.getDouble(1.0));

        m_shooterleader.configure(shooterleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      //for follower motor 
        SparkFlexConfig shooterfollowerConfig = new SparkFlexConfig();
        shooterfollowerConfig.idleMode(IdleMode.kCoast);
        shooterfollowerConfig.smartCurrentLimit(40);
        shooterfollowerConfig.follow(m_shooterleader); // follows leader motor
        shooterfollowerConfig.inverted(true); 
      
        m_shooterfollower.configure(shooterfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
    }
       //Shooter controls 

        //method to run the shooter 
        public void StartShooter(AngularVelocity velocity){
          if (velocity.lt(k_maxShooterRPM)) {
            m_controllerleader.setSetpoint(
            m_targetRPM, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            kFFEntry.getDouble(12.0 / 3000.0)
            );
          }
        }

        //method to stop the shooter 
        public void StopShooter() {
          m_controllerleader.setSetpoint(
            0, ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            0
            );
        }

        //live shuffleboard upates 
        public void updatePID(){
            double newP = kPEntry.getDouble(0.01);
            double newI = kIEntry.getDouble(0.0);
            double newD = kDEntry.getDouble(0.0);
            double newMin = minOutputEntry.getDouble(-1.0);
            double newMax = maxOutputEntry.getDouble(1.0);

            SparkFlexConfig config = new SparkFlexConfig();
            config.idleMode(IdleMode.kCoast);
            config.smartCurrentLimit(40);
            config.closedLoop.p(newP).i(newI).d(newD).outputRange(newMin, newMax);

            m_shooterleader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            k_maxShooterRPM = RPM.of(maxRPMEntry.getDouble(5767));
        }
      
        public boolean shooteratSpeed(){
          double currentRPM = m_shooterleader_encoder.getVelocity();
          double tolerance = 50;
          return Math.abs(currentRPM - m_targetRPM) <= tolerance;
        }

        //Shuffleboard Updates */ 
        @Override
        public void periodic() {
          
         updatePID();

         leaderRPMEntry.setDouble(m_shooterleader_encoder.getVelocity());
         followerRPMEntry.setDouble(m_shooterfollower_encoder.getVelocity());
     }
  }       
                       
            