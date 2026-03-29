package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


//using talonfx motor for feeder 
public class Feeder extends SubsystemBase
{
   //identifying motor 

   final TalonFX m_feeder_motor ;
   final TalonFXConfiguration m_feeder_encoder ; 
   final MotionMagicVelocityVoltage m_MagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
   
   final CommandSwerveDrivetrain m_pose_estimator;

   private Timer m_shootingtimer = new Timer();

   //shuffleboard tab entries 
     private final ShuffleboardTab tab = Shuffleboard.getTab("Feeder");
     private final GenericEntry kPEntry = tab.add("kP", 0.11).getEntry();
     private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
     private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
     private final GenericEntry kSEntry = tab.add("kS", 0.25).getEntry();
     private final GenericEntry kVEntry = tab.add("kV", 0.12).getEntry();
     private final GenericEntry kAEntry = tab.add("kA", 0.01).getEntry();
     private final GenericEntry targetVelocityEntry = tab.add("Target Velocity (RPS)", 0).getEntry();
     private final GenericEntry currentVelocityEntry = tab.add("Current Velocity (RPS)", 0).getEntry();
     private final GenericEntry motorTemperatureEntry = tab.add("Motor Temperature (C)", 0).getEntry();

     //intializing closed-loop target velocity
     private double targetVelocity = 0;

     //default feeder velocity 
     private static final double default_velocity = -60.0; //RPS - in consideration of shooter at 5767 RPM


      public Feeder(CANBus canbus, CommandSwerveDrivetrain pose_estimator){

         m_feeder_motor =new TalonFX(54, canbus);
         m_feeder_encoder = new TalonFXConfiguration(); 

         targetVelocity = 0;
         targetVelocityEntry.setDouble(targetVelocity);
         targetVelocityEntry.setDouble(default_velocity);

         configureMotor();

         m_pose_estimator = pose_estimator;
      }

      //creating the configuration process which will set limits for shooting
      private void configureMotor() {
         TalonFXConfiguration m_feeder_config = new TalonFXConfiguration();

      // set slot 0 gains + from phoneix tuner documentation 
      m_feeder_config.Slot0.kS = kSEntry.getDouble(0.25); // Add 0.25 V output to overcome static friction
      m_feeder_config.Slot0.kV = kVEntry.getDouble(0.12); // A velocity target of 1 rps results in 0.12 V output
      m_feeder_config.Slot0.kA = kAEntry.getDouble(0.01); // An acceleration of 1 rps/s requires 0.01 V output
      m_feeder_config.Slot0.kP = kPEntry.getDouble(0.11); // An error of 1 rps results in 0.11 V output
      m_feeder_config.Slot0.kI = kIEntry.getDouble(0.0);  // no output for integrated error
      m_feeder_config.Slot0.kD = kDEntry.getDouble(0.0);  // no output for error derivative
      
      //applying current limits to feeder 
      var talonFXConfigurator = m_feeder_motor.getConfigurator();
      var limitConfigs = new CurrentLimitsConfigs();
      
      //enable stator current limit 
      limitConfigs.StatorCurrentLimit = 100;
      limitConfigs.StatorCurrentLimitEnable = true;

      talonFXConfigurator.apply(limitConfigs);

      //magic motion settings 
      var motionMagicConfigs = m_feeder_config.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
      
      m_feeder_motor.getConfigurator().apply(m_feeder_config);
      
      MotorOutputConfigs feederOutput = new MotorOutputConfigs();
      feederOutput.NeutralMode = NeutralModeValue.Brake;
      feederOutput.DutyCycleNeutralDeadband = 0.02; 
      m_feeder_motor.getConfigurator().apply(feederOutput);

      

      
      }

      //method to start the feeder when shooter activates 
      public void startFeeder() {
         targetVelocity = default_velocity;
         m_shootingtimer.start();
         m_shooting_reset_timer.reset();
         m_shooting_reset_timer.stop();

         targetVelocityEntry.setDouble(targetVelocity);
         m_feeder_motor.setControl(m_MagicVelocityVoltage.withVelocity(default_velocity));
         
      }
      
      //method to stop the feeder when shooter stops
      public void stopFeeder() {
         targetVelocity = 0;

         m_shootingtimer.stop();
         m_shooting_reset_timer.start();
         
         m_feeder_motor.setControl(m_MagicVelocityVoltage.withVelocity(0));
         targetVelocityEntry.setDouble(0);
      }

      public boolean withShooter(){
         double currentVel =
                  m_feeder_motor.getRotorVelocity().getValueAsDouble();
               return Math.abs(currentVel - targetVelocity) < 2.0;
      }

    
      public Command auto_feeder_start(){

         return Commands.sequence(
            new InstantCommand(this::startFeeder,this),
            new WaitCommand(0.25)
         );
         
      }

      public Command auto_feeder_end(){

         return Commands.sequence(
            new InstantCommand(this::stopFeeder, this),
            new WaitCommand(0.25)
         );
      }
      public boolean been_shooting(double time){
          return m_shootingtimer.get()>time;
        }

      //Shuffleboard Updates */ 

      Timer m_shooting_reset_timer = new Timer();
      @Override
      public void periodic() {

          //running motion magic control
         //  m_feeder_motor.setControl(
         //       new MotionMagicVelocityVoltage(targetVelocity));
          
          //updating current velocity 
          currentVelocityEntry.setDouble(
                  m_feeder_motor.getRotorVelocity().getValueAsDouble());
          motorTemperatureEntry.setDouble(m_feeder_motor.getDeviceTemp().getValueAsDouble());

          if (m_shooting_reset_timer.get()>5){
            m_shootingtimer.reset();
          }


      }
      
   }
         



      
      



