package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.VelocityUnit;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;



//using talonfx motor for feeder 
public class Intake extends SubsystemBase
{
   //identifying motor 

   final TalonFX m_intake_roller_motor ;
   final TalonFXConfiguration m_intake_roller_config;

   final MotionMagicVelocityVoltage m_MagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
   
   final TalonFX m_intake_arm_motor;
   final TalonFXConfiguration m_intake_arm_config;

   
   final Angle arm_start = Rotations.of(1.39);
   final Angle arm_end = Rotations.of(57.917-4);
   final Angle arm_delta = arm_end.minus(arm_start);

   final MotionMagicVoltage m_position_request = new MotionMagicVoltage(0);
  
   final CommandSwerveDrivetrain m_pose_estimator;

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

      public Intake(CommandSwerveDrivetrain pose_estimator){

         m_intake_roller_motor =new TalonFX(53);
         m_intake_roller_config = new TalonFXConfiguration();
         targetVelocity = 0;
         targetVelocityEntry.setDouble(targetVelocity);
         targetVelocityEntry.setDouble(default_velocity);


         m_intake_arm_motor = new TalonFX( 52);
         m_intake_arm_config = new TalonFXConfiguration();

         m_intake_arm_motor.setPosition(0);

     

         configureMotor();

         m_pose_estimator = pose_estimator;
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

      public void reverseRoller(){
         targetVelocity = -default_velocity;
         m_intake_roller_motor.setControl(m_MagicVelocityVoltage.withVelocity(-default_velocity));
      }
      
      private boolean roller_on =  false;
      
      
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
      //   System.out.println("arm out");
      }

      //extending in intake
      public void retractArm(){
        m_intake_arm_motor.setControl(m_position_request.withPosition(0));
      //   System.out.println("arm in");
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
      
      public void active_wiggle(double wiggle){

         Angle out_wiggle = Rotations.of(57).minus(arm_delta);
         Angle in_wiggle = Rotations.of(32);

         Angle adjust = wiggle<0 ? out_wiggle.times(wiggle) : in_wiggle.times(wiggle);

         if (arm_out){
            m_intake_arm_motor.setControl(m_position_request.withPosition(arm_delta.minus(adjust)));
         }
   



      }
      public void half_deploy(){
         m_intake_arm_motor.setControl(m_position_request.withPosition(arm_delta.times(0.5)));
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
      




      // AUTO COMMANDS + RELEVANT CODE BELOW

          private void set_intake_speed(AngularVelocity speed){

        // if (speed.gt(k_max_wheel_speed)){
        //     //logger.log("max wheel speed exceeded")
        //     speed = k_max_wheel_speed;

        // }
        // else if(speed.lt(k_max_wheel_speed.unaryMinus())){//unary Minus is negate
        //     //logger.log("negativce max wheel speed exceeded")
        //     speed = k_max_wheel_speed.unaryMinus();
        // }

        m_intake_roller_motor.setControl(m_intakeFXOut_v_mm.withVelocity(speed));

          }

         private void stop_intake(){
        m_intake_roller_motor.setControl(m_brake);
    }

   private final NeutralOut m_brake = new NeutralOut();

    public boolean m_has_fuel = false;
        public BooleanSupplier has_fuel(){

        return ()-> m_has_fuel;
    }
        

   final MotionMagicVelocityTorqueCurrentFOC m_intakeFXOut_v_mm = new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);
   final MotionMagicExpoTorqueCurrentFOC m_intakeFXOut_ep_mm = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);


      private void has_fuel_false(){
         m_has_fuel = false;
      }


   public Command auto_intake_fuel_command(){

         return run(()->{
         set_intake_speed(AngularVelocity.ofBaseUnits(80, RotationsPerSecond));})
         .withTimeout(2);
         // .andThen(this::stop_intake)
         // .andThen(this::has_fuel_false);
      }

   public Command auto_intake_fuel_stop(){
      return run(()->{
         set_intake_speed(AngularVelocity.ofBaseUnits(0, RotationsPerSecond));
      });
   }

   public Command auto_outtake_fuel_command(){

        return run(()->{set_intake_speed(AngularVelocity.ofBaseUnits(-80, RotationsPerSecond));})
            .withTimeout(2)
            .andThen(this::stop_intake)
            .andThen(this::has_fuel_false);

   }

   public Command auto_extract_out_intake_command(){

      return Commands.sequence(
         new InstantCommand(this::extendArm,this),
         new WaitCommand(1)
      );


   }

   public Command auto_retract_in_intake_command(){

      return Commands.sequence(
         new InstantCommand(this::retractArm,this),
         new WaitCommand(1)

      );
   }


       public void publish_intake_data(){

        // put data important for charaterizing the data to the smart dashboard
        double set_point = m_intake_roller_motor.getClosedLoopReference().getValueAsDouble();
        double error = m_intake_roller_motor.getClosedLoopError().getValueAsDouble();
        double tcurrent = m_intake_roller_motor.getTorqueCurrent().getValueAsDouble();
        double velocity = m_intake_roller_motor.getVelocity().getValueAsDouble();
        double acceleration = m_intake_roller_motor.getAcceleration().getValueAsDouble();
        double position = m_intake_roller_motor.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Intake Set Point", set_point);
        SmartDashboard.putNumber("Intake Error", error);
        SmartDashboard.putNumber("Intake Torque Current", tcurrent);
        SmartDashboard.putNumber("Intake Velocity", velocity);
        SmartDashboard.putNumber("Intake Acceleration", acceleration);
        SmartDashboard.putNumber("Intake Position", position);

        
    }
   }
         



      
      



