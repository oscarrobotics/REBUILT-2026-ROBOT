package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//using talonfx motor for feeder 
public class Intake extends SubsystemBase
{
   //identifying motor 

   final TalonFX m_intake_roller_motor ;
   final TalonFXConfiguration m_intake_roller_config;

   final MotionMagicVelocityVoltage m_MagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
   
   final TalonFX m_intake_arm_motor;
   final TalonFXConfiguration m_intake_arm_config;

   
   final Angle arm_start = Rotations.of(-7.3666);
   final Angle arm_end = Rotations.of(50.1372);
   final Angle arm_delta = arm_end.minus(arm_start);

   final MotionMagicVoltage m_position_request = new MotionMagicVoltage(0);
  


   //shuffleboard tab entries 
     private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
     private final GenericEntry kPEntry = tab.add("kP", 0.11).getEntry();
     private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
     private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
     private final GenericEntry kSEntry = tab.add("kS", 0.25).getEntry();
     private final GenericEntry kVEntry = tab.add("kV", 0.12).getEntry();
     private final GenericEntry kAEntry = tab.add("kA", 0.01).getEntry();
     private final GenericEntry targetVelocityEntry = tab.add("Target Velocity (RPS)", 0).getEntry();
     private final GenericEntry currentVelocityEntry = tab.add("Current Velocity (RPS)", 0).getEntry();

     //intializing closed-loop target velocity
     private double targetVelocity = 0;

     //default feeder velocity 
     private static final double default_velocity = 80.0;

      public Intake(){

         m_intake_roller_motor =new TalonFX(53);
         m_intake_roller_config = new TalonFXConfiguration();
         targetVelocity = 0;
         targetVelocityEntry.setDouble(targetVelocity);
         targetVelocityEntry.setDouble(default_velocity);


         m_intake_arm_motor = new TalonFX( 52);
         m_intake_arm_config = new TalonFXConfiguration();

         m_intake_arm_motor.setPosition(0);

     

         configureMotor();
      }

      //creating the configuration process which will set limits for shooting
      private void configureMotor() {
         TalonFXConfiguration m_intake_roller_config = new TalonFXConfiguration();

      // set slot 0 gains + from phoneix tuner documentation 
      m_intake_roller_config.Slot0.kS = kSEntry.getDouble(0.25); // Add 0.25 V output to overcome static friction
      m_intake_roller_config.Slot0.kV = kVEntry.getDouble(0.12); // A velocity target of 1 rps results in 0.12 V output
      m_intake_roller_config.Slot0.kA = kAEntry.getDouble(0.01); // An acceleration of 1 rps/s requires 0.01 V output
      m_intake_roller_config.Slot0.kP = kPEntry.getDouble(0.11); // An error of 1 rps results in 0.11 V output
      m_intake_roller_config.Slot0.kI = kIEntry.getDouble(0.0);  // no output for integrated error
      m_intake_roller_config.Slot0.kD = kDEntry.getDouble(0.0);  // no output for error derivative
      
      //magic motion settings 
      var motionMagicConfigs = m_intake_roller_config.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
      
      m_intake_roller_motor.getConfigurator().apply(m_intake_roller_config);
      
      MotorOutputConfigs intake_roller_Output = new MotorOutputConfigs();
      intake_roller_Output.NeutralMode = NeutralModeValue.Coast;
      intake_roller_Output.DutyCycleNeutralDeadband = 0.02; 
      m_intake_roller_motor.getConfigurator().apply(intake_roller_Output);



       // set slot 0 gains + from phoneix tuner documentation 
   
    // m_intake_arm_config.Slot0.kS = kSEntry.getDouble(0.25); // Add 0.25 V output to overcome static friction
    m_intake_arm_config.Slot0.kV = kVEntry.getDouble(0.22); // A velocity target of 1 rps results in 0.12 V output
    m_intake_arm_config.Slot0.kA = kAEntry.getDouble(0.02); // An acceleration of 1 rps/s requires 0.01 V output
    m_intake_arm_config.Slot0.kP = kPEntry.getDouble(0.21); // An error of 1 rps results in 0.11 V output
    m_intake_arm_config.Slot0.kI = kIEntry.getDouble(0.001);  // no output for integrated error
    m_intake_arm_config.Slot0.kD = kDEntry.getDouble(0.0);  // no output for error derivative

    m_intake_arm_config.CurrentLimits.StatorCurrentLimit = 30;

    //magic motion settings 
    var arm_motionMagicConfigs = m_intake_arm_config.MotionMagic;
    arm_motionMagicConfigs.MotionMagicCruiseVelocity = 80;
    arm_motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 400 rps/s (0.25 seconds to max)
    arm_motionMagicConfigs.MotionMagicJerk = 2000; // Target jerk of 4000 rps/s/s (0.1 seconds)
    
    m_intake_arm_motor.getConfigurator().apply(m_intake_arm_config);
    
    MotorOutputConfigs intake_arm_Output = new MotorOutputConfigs();
    intake_arm_Output.NeutralMode = NeutralModeValue.Coast;

    intake_arm_Output.DutyCycleNeutralDeadband = 0.02; 
    m_intake_arm_motor.getConfigurator().apply(intake_arm_Output);

   
    
    }

      //method to start the feeder rolls
      public void startRoller() {
         targetVelocity = default_velocity;
         targetVelocityEntry.setDouble(targetVelocity);
         m_intake_roller_motor.setControl(m_MagicVelocityVoltage.withVelocity(default_velocity));
         
      }
      
      //method to stop the feeder rolls
      public void stopRoller() {
         targetVelocity = 0;
         m_intake_roller_motor.setControl(m_MagicVelocityVoltage.withVelocity(0));
         targetVelocityEntry.setDouble(0);
      }
      
      private boolean roller_on =  false;
      
      //command to simultanouesly extract and retract intake arm
      public void toggle_roller(){

        if (roller_on){
            roller_on = false;
            this.stopRoller();
        }
        else{
            roller_on = true;
            this.startRoller();
        }

      }

      // public void startIntake() {
      //    targetVelocity = default_velocity;
      //    targetVelocityEntry.setDouble(targetVelocity);
      //    m_intake_motor.setControl(m_MagicVelocityVoltage.withVelocity(default_velocity));
      // }

      // public void stopIntake() {
      //     targetVelocity = 0;
      //    m_intake_motor.setControl(m_MagicVelocityVoltage.withVelocity(0));
      //    // m_intake_arm_motor.setControl(m_)
      //    targetVelocityEntry.setDouble(0);
      // }

      //extending out intake 
      public void extendArm(){
        m_intake_arm_motor.setControl(m_position_request.withPosition(arm_delta));
        System.out.println("arm out");
      }

      //extending in intake
      public void retractArm(){
        m_intake_arm_motor.setControl(m_position_request.withPosition(0));
        System.out.println("arm in");
      }
      private boolean arm_out =  false;
      
      //command to simultanouesly extract and retract intake arm
      public void toggle_arm(){

        if (arm_out){
            arm_out = false;
            this.retractArm();
        }
        else{
            arm_out = true;
            this.extendArm();
        }


      }

      // public void bounce_arm(){
      //    if(m_ar)
      //    if (m_intake_arm_motor.getPosition().isNear(arm_delta.in(Rotations), 1)){
      //       m_intake_arm_motor.setControl(m_position_request.withPosition(arm_delta.minus(Rotations.of(5))));
      //    }
      //    if 

      // }
                                

      //Shuffleboard Updates */ 
      @Override
      public void periodic() {

      }
      
   }
         



      
      



