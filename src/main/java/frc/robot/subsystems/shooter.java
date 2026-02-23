package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;


public class Shooter extends SubsystemBase{
    
    //identifying motors + encoders with their ids 
    private final SparkFlex m_shooterleader = new SparkFlex(6, MotorType.kBrushless); //RIGHT (robot facing front)
    private final SparkFlex m_shooterfollower = new SparkFlex(7, MotorType.kBrushless); //LEFT SIDE (robot facing front)
    
    private final RelativeEncoder m_shooterleader_encoder = m_shooterleader.getEncoder();
    private final RelativeEncoder m_shooterfollower_encoder = m_shooterfollower.getEncoder();

    // Initialize the closed loop controller -> using closed loop control system for shooter mechanism 
    private final SparkClosedLoopController m_controllerfollower = m_shooterfollower.getClosedLoopController();
    private final SparkClosedLoopController m_controllerleader = m_shooterleader.getClosedLoopController();

    private static final double TARGET_RPM = 3000.0; //target RPM for motors (make sure to update if needed)


         // SHUFFLEBOARD PID ENTRIES + DEFAULT VALUES (can change on shuffleboard, rather than adjusting code 24/7)
        private GenericEntry kPEntry;
        private GenericEntry kIEntry;
        private GenericEntry kDEntry;
        private GenericEntry kFEntry; 
        
        private static final double kP = 0.1;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double kF = 0.0;
        private static final double kMinOutput = -1.0;
        private static final double kMaxOutput = 1.0;


  
    public Shooter(){
        configureMotors();
        
    }

    //creating the configuration process which will set limits for shooting + adjustments/tuning for velocity 
                
    private void configureMotors(){
      //for leader motor 
      SparkFlexConfig shooterleaderConfig = new SparkFlexConfig();
        shooterleaderConfig.idleMode(IdleMode.kCoast);
        shooterleaderConfig.smartCurrentLimit(40);
        shooterleaderConfig.closedLoop.p(kP).i(kI).d(kD);
        shooterleaderConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
        m_shooterleader.configure(shooterleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      //for follower motor 
        SparkFlexConfig shooterfollowerConfig = new SparkFlexConfig();
        shooterfollowerConfig.idleMode(IdleMode.kCoast);
        shooterfollowerConfig.smartCurrentLimit(40);
        shooterfollowerConfig.follow(m_shooterleader); // follows leader motor
        shooterfollowerConfig.inverted(true); 
        m_shooterfollower.configure(shooterfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

      //creating shooter shuffleboard tab 
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        
        //adding PID gains to shuffleboard 
        kPEntry = tab.add("P Gain", kP).getEntry();
        kIEntry = tab.add("I Gain", kI).getEntry();
        kDEntry = tab.add("D Gain", kD).getEntry();
        kFEntry = tab.add("F Gain", kF).getEntry(); 

        setPIDConstants();



    }
        private void setPIDConstants() {

          // PID constants from shuffleboard entries to both leader and follower motor 
          double kP = kPEntry.getDouble(0.1);
          double kI = kIEntry.getDouble(0.0);
          double kD = kDEntry.getDouble(0.0);
          double kF = kFEntry.getDouble(0.0);

          m_shooterleader.set(kP);
          m_shooterleader.set(kI);
          m_shooterleader.set(kD);
          m_shooterleader.set(kF);
          
          m_shooterfollower.set(kP);
          m_shooterfollower.set(kI);
          m_shooterfollower.set(kD);
          m_shooterfollower.set(kF);

        }

        //command to run the shooter 
        public Command Startshooter(){
          return new InstantCommand(() -> {
            m_shooterleader.set(TARGET_RPM / 5700.0); 
            m_shooterfollower.set(TARGET_RPM / 5700.0);
          });
        }

        //command to stop the shooter 
        public Command stopShooter(){
          return new InstantCommand(() -> {
            m_shooterleader.set(0);
            m_shooterfollower.set(0);
          });
        }
        
      
        /**SmartDashboard Updates */
        @Override
        public void periodic() {
          setPIDConstants();
          SmartDashboard.putNumber("Shooter Follower RPM", m_shooterfollower_encoder.getVelocity());
          SmartDashboard.putNumber("Shooter Leader RPM", m_shooterleader_encoder.getVelocity());
        }

        }
        
                       
            