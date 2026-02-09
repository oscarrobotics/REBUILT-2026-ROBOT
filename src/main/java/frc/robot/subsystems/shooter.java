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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//using talonfx motor for shooter 
public class Shooter extends SubsystemBase
{
   //identifying motor + encoder with their ids 
   final TalonFX m_shooter_motor = new TalonFX(0); //placeholder id 
   final CANcoder m_shooter_CaNcoder = new CANcoder(1); //placeholder id
   private final VoltageOut m_VoltageOut = new VoltageOut(0); 

      public Shooter(){
      //creating the configuration process which will set limits for shooting
      var m_shooter_config = new TalonFXConfiguration();

      // set slot 0 gains + from phoneix tuner documentation 
      var slot0Configs = m_shooter_config.Slot0; 
      slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0; // no output for error derivative

      // set Motion Magic Velocity settings + from phoneix tuner documentation 
      var motionMagicConfigs = m_shooter_config.MotionMagic;
      motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
      motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

      m_shooter_motor.getConfigurator().apply(m_shooter_config);


      // create a Motion Magic Velocity request, voltage output + from phoneix tuner documentation 
      final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

      //placeholder button 
      //if (m_joy.getAButton()) {
   // while the joystick A button is held, use a slower acceleration
  // m_request.Acceleration = 100; // rot/s^2
      // } else {
       // otherwise, fall back to the config
      // m_request.Acceleration = 0;
     //  }

     // set target velocity to 80 rps
     m_shooter_motor.setControl(m_request.withVelocity(80));

      }
   

      
   }
         



      
      



