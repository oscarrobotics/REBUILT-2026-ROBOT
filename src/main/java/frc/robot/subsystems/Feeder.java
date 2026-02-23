package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;


//using talonfx motor for feeder 
public class Feeder extends SubsystemBase
{
   //identifying motor + encoder with their ids 
   final TalonFX m_feeder_motor = new TalonFX(5);  
   //final m_feeder_motor.setControl(new DutyCycleOut(1.0)); // 100% full speed positive.
   // final CANcoder m_feeder_CaNcoder = new CANcoder(1); //placeholder id
   private final VoltageOut m_VoltageOut = new VoltageOut(0); 
   private final MotionMagicVelocityVoltage m_speedRequest = new MotionMagicVelocityVoltage(0);


   private double kP = SmartDashboard.getNumber("Feeder kP", 0.25);
   private double kD = SmartDashboard.getNumber("Feeder kD", 0);
   private double kI = SmartDashboard.getNumber("Feeder kI", 0);
   private double kS = SmartDashboard.getNumber("Feeder kS", 0.25);
   private double kV = SmartDashboard.getNumber("Feeder kV", 0.12);
   private double kA = SmartDashboard.getNumber("Feeder kA", 0.01);



   public Feeder(){
      //creating the configuration process which will set limits for shooting
      var m_feeder_config = new TalonFXConfiguration();

      // set slot 0 gains + from phoneix tuner documentation 
      var slot0Configs = m_feeder_config.Slot0; 
      slot0Configs.kS = kS; // Add 0.25 V output to overcome static friction
      slot0Configs.kV = kV; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = kA; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = kP; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = kI; // no output for integrated error
      slot0Configs.kD = kD; // no output for error derivative
      // set Motion Magic Velocity settings + from phoneix tuner documentation 
      var motionMagicConfigs = m_feeder_config.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

      m_feeder_motor.getConfigurator().apply(m_feeder_config);


  

   
      
    

   }
   
   public void feed(){
      m_feeder_motor.setControl(m_speedRequest.withVelocity(10)); 
   }

   public void stop_feeding(){
      m_feeder_motor.setControl(m_VoltageOut.withOutput(0)); 
   } 
   
   public SequentialCommandGroup feed_command() {
      return new RunCommand(() -> feed(), this).andThen(() -> stop_feeding());
   }

   @Override
   public void periodic() {
      // This method will be called once per scheduler run    
      // update the smart dashboard with the current velocity of the feeder motor 
      SmartDashboard.putNumber("Feeder Velocity (RPM)", m_feeder_motor.getVelocity().getValue().in(RPM));

      double new_kP = SmartDashboard.getNumber("Feeder kP", kP);
      double new_kI = SmartDashboard.getNumber("Feeder kI", kI);
      double new_kD = SmartDashboard.getNumber("Feeder kD", kD);
      double new_kS = SmartDashboard.getNumber("Feeder kS", kS);
      double new_kV = SmartDashboard.getNumber("Feeder kV", kV);
      double new_kA = SmartDashboard.getNumber("Feeder kA", kA);

      // Check for PID constant updates from SmartDashboard
      if (new_kP != kP || new_kI != kI || new_kD != kD || new_kS != kS || new_kV != kV || new_kA != kA) {
          kP = new_kP;
          kI = new_kI;
          kD = new_kD;
          kS = new_kS;
          kV = new_kV;
          kA = new_kA;

          // Update the motor configuration with the new PID constants
          var updatedConfig = new TalonFXConfiguration();
          var slot0Configs = updatedConfig.Slot0; 
          slot0Configs.kS = kS; 
          slot0Configs.kV = kV; 
          slot0Configs.kA = kA; 
          slot0Configs.kP = kP; 
          slot0Configs.kI = kI; 
          slot0Configs.kD = kD; 

          m_feeder_motor.getConfigurator().apply(updatedConfig);
      }  

   }


   

      
}
         



      
      



