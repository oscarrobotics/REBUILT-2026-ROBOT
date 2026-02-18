package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

public class Shooter extends SubsystemBase{
    
    //identifying motors + encoders with their ids (motor names will change once fully mounted)
       private final SparkFlex m_shooterleader = new SparkFlex(6, MotorType.kBrushless); //RIGHT (robot facing front)
       private final SparkFlex m_shooterfollower = new SparkFlex(7, MotorType.kBrushless); //LEFT SIDE (robot facing front)
       
       private final RelativeEncoder m_shooterleader_encoder = m_shooterleader.getEncoder();
       private final RelativeEncoder m_shooterfollower_encoder = m_shooterfollower.getEncoder();

       private SparkClosedLoopController m_SparkClosedLoopController; 
      
       // Initialize the closed loop controller -> using closed loop control system for shooter mechanism 
       SparkClosedLoopController m_controllerfollower = m_shooterfollower.getClosedLoopController();
       SparkClosedLoopController m_controllerleader = m_shooterleader.getClosedLoopController();

          //PID constants (subject to change - can change via smartdashboard)
          private double kP = SmartDashboard.getNumber("Shooter kP", 0.01);
          private double kD = SmartDashboard.getNumber("Shooter kD", 0);
          private double kI = SmartDashboard.getNumber("Shooter kI", 0);
          private double kFF = SmartDashboard.getNumber("Shooter kFF", 0.00015);
          private double kMaxOutput = SmartDashboard.getNumber("Shooter kMaxOutput", 1.0);
          private double kMinOutput = SmartDashboard.getNumber("Shooter kMinOutput", -1.0);
          private double maxRPM = SmartDashboard.getNumber("Shooter RPM", 4000.0);
          
        
          private static final double shooterleader_RPM = 4000;


    

          
    
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
             shooterfollowerConfig.follow(m_shooterleader); // follows left motor
             shooterfollowerConfig.inverted(true); 
             m_shooterfollower.configure(shooterfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




          }
        
        
        public void ShooterVelocity(AngularVelocity velocity){
          m_shooterleader.set(velocity.in(RPM));


        }

        public void stopShooter() {
          m_shooterfollower.set(0);
          m_shooterleader.set(0);
        }

        /**SmartDashboard Updates */
        @Override
        public void periodic() {
          SmartDashboard.putNumber("Shooter Follower RPM", m_shooterfollower_encoder.getVelocity());
          SmartDashboard.putNumber("Shooter Leader RPM", m_shooterleader_encoder.getVelocity());
        }

        }
        
                       
            