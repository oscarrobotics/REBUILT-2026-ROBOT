package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


//using talonfx motor for feeder 
public class Hopper extends SubsystemBase
{
   //identifying motor 

   final TalonFX m_hopper_motor ;
   final TalonFXConfiguration m_hopper_encoder ; 
   final MotionMagicVelocityVoltage m_MagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
   
  
   //shuffleboard tab entries 
     private final ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
     private final GenericEntry kPEntry = tab.add("kP", 0.11).getEntry();
     private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
     private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
     private final GenericEntry kSEntry = tab.add("kS", 0.25).getEntry();
     private final GenericEntry kVEntry = tab.add("kV", 0.12).getEntry();
     private final GenericEntry kAEntry = tab.add("kA", 0.01).getEntry();
     private final GenericEntry targetVelocityEntry = tab.add("Target Velocity (RPS)", 0).getEntry();
     private final GenericEntry currentVelocityEntry = tab.add("Current Velocity (RPS)", 0).getEntry();

     //intializing closed-loop target velocity
     private AngularVelocity targetVelocity = RPM.of(0);

     //default feeder velocity 
     private static final AngularVelocity default_velocity = RotationsPerSecond.of(-50.0); //RPS - in consideration of shooter at 5767 RPM (changed from -20 to -50)


    public Hopper(){

         m_hopper_motor =new TalonFX(51);
         m_hopper_encoder = new TalonFXConfiguration(); 

        
         targetVelocityEntry.setDouble(targetVelocity.in(RPM));
         targetVelocityEntry.setDouble(default_velocity.in(RPM));

         configureMotor();
      }

      //creating the configuration process which will set limits for shooting
      private void configureMotor() {
         TalonFXConfiguration m_feeder_config = new TalonFXConfiguration();

      // set slot 0 gains + from phoneix tuner documentation 
      m_feeder_config.Slot0.kS = kSEntry.getDouble(0.25); // Add 0.25 V output to overcome static friction
      m_feeder_config.Slot0.kV = kVEntry.getDouble(0.12); // A velocity target of 1 rps results in 0.12 V output
      m_feeder_config.Slot0.kA = kAEntry.getDouble(0.01); // An acceleration of 1 rps/s requires 0.01 V output
      m_feeder_config.Slot0.kP = kPEntry.getDouble(0.11); // An error of 1 rps results in 0.11 V output
      m_feeder_config.Slot0.kI = kIEntry.getDouble(0.01);  // no output for integrated error
      m_feeder_config.Slot0.kD = kDEntry.getDouble(0.01);  // no output for error derivative
      
      //magic motion settings 
      var motionMagicConfigs = m_feeder_config.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
      
      m_hopper_motor.getConfigurator().apply(m_feeder_config);
      
      MotorOutputConfigs feederOutput = new MotorOutputConfigs();
      feederOutput.NeutralMode = NeutralModeValue.Brake;
      feederOutput.DutyCycleNeutralDeadband = 0.02; 
      m_hopper_motor.getConfigurator().apply(feederOutput);
      
      }

      //method to start the feeder when shooter activates 
      public void startHopper() {
         targetVelocity = default_velocity;
         targetVelocityEntry.setDouble(targetVelocity.in(RPM));
         m_hopper_motor.setControl(m_MagicVelocityVoltage.withVelocity(default_velocity));
         
      }
      
      //method to stop the feeder when shooter stops
      public void stopHopper() {
         targetVelocity = RPM.of(0);
         m_hopper_motor.setControl(m_MagicVelocityVoltage.withVelocity(0));
         targetVelocityEntry.setDouble(0);
      }

      public void reverseHopper(){
         targetVelocity = default_velocity.times(-0.5);

         m_hopper_motor.setControl(m_MagicVelocityVoltage.withVelocity(targetVelocity));

         targetVelocityEntry.setDouble(targetVelocity.in(RPM));
      }

      public Command auto_start_hopper(){

         return Commands.sequence(
            new InstantCommand(this::startHopper, this),
            new WaitCommand(1)
         );
      }

      public Command auto_stop_hopper(){

         return Commands.sequence(
            new InstantCommand(this::stopHopper,this),
            new WaitCommand(1)
         );
      }

      public Command auto_reverse_hopper(){

         return Commands.sequence(
            new InstantCommand(this::reverseHopper, this),
            new WaitCommand(1)
         );
      }
    

    

      //Shuffleboard Updates */ 
      @Override
      public void periodic() {

          //running motion magic control
          //updating current velocity 
          currentVelocityEntry.setDouble(
                  m_hopper_motor.getRotorVelocity().getValueAsDouble());
          
      }
      
   }
         



      
      



