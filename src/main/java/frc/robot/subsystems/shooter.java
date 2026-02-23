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
  private final SparkFlex m_shooterleader;
  private final SparkFlex m_shooterfollower;
  
  private final RelativeEncoder m_shooterleader_encoder;
  private final RelativeEncoder m_shooterfollower_encoder;

  private final SparkFlexConfig m_shooterleaderConfig;
  private final SparkFlexConfig m_shooterfollowerConfig;
  

  // Initialize the closed loop controller -> using closed loop control system for shooter mechanism 
  // SparkClosedLoopController m_controllerfollower = m_shooterfollower.getClosedLoopController();
  private SparkClosedLoopController m_controllerleader;

  //PID constants (subject to change - can change via smartdashboard)
  private double kP = SmartDashboard.getNumber("Shooter kP", 0.01);
  private double kD = SmartDashboard.getNumber("Shooter kD", 0);
  private double kI = SmartDashboard.getNumber("Shooter kI", 0);
  private double kFF = SmartDashboard.getNumber("Shooter kFF", 12.0 / 5767.0); // feedforward constant based on max voltage and max RPM of the motors
  private double kMaxOutput = SmartDashboard.getNumber("Shooter kMaxOutput", 1.0);
  private double kMinOutput = SmartDashboard.getNumber("Shooter kMinOutput", -1.0);

        
  public AngularVelocity k_maxShooterRPM ; // max RPM of the motors (subject to change based on testing)


    

          
    
    public Shooter(){

        m_shooterleader = new SparkFlex(6, MotorType.kBrushless); //RIGHT (robot facing front)
        m_shooterfollower = new SparkFlex(7, MotorType.kBrushless); //LEFT SIDE (robot facing front)

        m_shooterleader_encoder = m_shooterleader.getEncoder();
        m_shooterfollower_encoder = m_shooterfollower.getEncoder();

        m_controllerleader = m_shooterleader.getClosedLoopController();

        m_shooterleaderConfig = new SparkFlexConfig();
        m_shooterfollowerConfig = new SparkFlexConfig();

        k_maxShooterRPM = RPM.of(SmartDashboard.getNumber("Shooter max RPM", 5767.0));

        configureMotors();
    }
    //creating the configuration process which will set limits for shooting + adjustments/tuning for velocity 
                
    private void configureMotors(){
      //for leader motor 
      
        m_shooterleaderConfig.idleMode(IdleMode.kCoast);
        m_shooterleaderConfig.smartCurrentLimit(40);
        m_shooterleaderConfig.closedLoop.p(kP).i(kI).d(kD).feedForward.kV(kFF);
        m_shooterleaderConfig.closedLoop.outputRange(kMinOutput, kMaxOutput);
        m_shooterleader.configure(m_shooterleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      //for follower motor 
        
        m_shooterfollowerConfig.idleMode(IdleMode.kCoast);
        m_shooterfollowerConfig.smartCurrentLimit(40);
        m_shooterfollowerConfig.follow(m_shooterleader); // follows left motor
        m_shooterfollowerConfig.inverted(true); 
        m_shooterfollower.configure(m_shooterfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);




    }
  
  
    public void ShooterVelocity(AngularVelocity velocity){

      if (velocity.lt(k_maxShooterRPM)) {
        m_controllerleader.setSetpoint(velocity.in(RPM), ControlType.kVelocity); // Set the desired velocity in RPM  
      }


    }

    public void stopShooter() {
      
      m_controllerleader.setSetpoint(0, ControlType.kVelocity); // Stop the shooter by setting duty cycle to 0
    }

    /**SmartDashboard Updates */
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter Follower RPM", m_shooterfollower_encoder.getVelocity());
      SmartDashboard.putNumber("Shooter Leader RPM", m_shooterleader_encoder.getVelocity());




      // Check for PID constant updates from SmartDashboard
      boolean updatePID = false;
      
      double newkP = SmartDashboard.getNumber("Shooter kP", kP);
      double newkI = SmartDashboard.getNumber("Shooter kI", kI);
      double newkD = SmartDashboard.getNumber("Shooter kD", kD);
      double newkFF = SmartDashboard.getNumber("Shooter kFF", kFF);
      
      if (newkP != kP) {
        kP = newkP;
        updatePID = true;
        m_shooterleaderConfig.closedLoop.p(kP);
      } 
      if (newkI != kI) {
        kI = newkI;
        updatePID = true;
        m_shooterleaderConfig.closedLoop.i(kI);
      }
      if (newkD != kD) {
        kD = newkD;
        updatePID = true;
        m_shooterleaderConfig.closedLoop.d(kD);
      }
      if (newkFF != kFF) {
        kFF = newkFF;
        updatePID = true;
        m_shooterleaderConfig.closedLoop.feedForward.kV(kFF);
      }
      if (updatePID) {
        m_shooterleader.configure(m_shooterleaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      } 


      



    }

  }
        
                       
            