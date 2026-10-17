package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Intake rollers: one TalonFX leader (53) and one opposed follower (55). */
public class IntakeRoller extends SubsystemBase {

    private static final int k_leaderId = 53;
    private static final int k_followerId = 55;

    // Roller speed in rotations per second.
    private static final double k_defaultVelocity = 100; // original value 70
    private static final double k_outtakeVelocity = -80.0;

    private final TalonFX m_leader = new TalonFX(k_leaderId);
    private final TalonFX m_follower = new TalonFX(k_followerId);

    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);

    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Intake Roller");
    private final GenericEntry kPEntry = tab.add("kP", 0.11).getEntry();
    private final GenericEntry kIEntry = tab.add("kI", 0.0).getEntry();
    private final GenericEntry kDEntry = tab.add("kD", 0.0).getEntry();
    private final GenericEntry kSEntry = tab.add("kS", 0.25).getEntry();
    private final GenericEntry kVEntry = tab.add("kV", 0.12).getEntry();
    private final GenericEntry kAEntry = tab.add("kA", 0.01).getEntry();
    private final GenericEntry targetVelocityEntry = tab.add("Target Velocity (RPS)", 0).getEntry();
    private final GenericEntry currentVelocityEntry = tab.add("Current Velocity (RPS)", 0).getEntry();
    private final GenericEntry temperatureEntry = tab.add("Roller Motor Temperature (C)", 0).getEntry();

    private double m_targetVelocity = 0;
    private boolean m_rollerOn = false;

    public IntakeRoller() {
        configureMotors();
        targetVelocityEntry.setDouble(0);
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kS = kSEntry.getDouble(0.40);
        config.Slot0.kV = kVEntry.getDouble(0.12);
        config.Slot0.kA = kAEntry.getDouble(0.01);
        config.Slot0.kP = kPEntry.getDouble(0.11);
        config.Slot0.kI = kIEntry.getDouble(0.0);
        config.Slot0.kD = kDEntry.getDouble(0.0);

        config.CurrentLimits.StatorCurrentLimit = 70;

        config.MotionMagic.MotionMagicAcceleration = 100;
        config.MotionMagic.MotionMagicJerk = 1000;

        m_leader.getConfigurator().apply(config);

        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = NeutralModeValue.Coast;
        output.DutyCycleNeutralDeadband = 0.02;
        m_leader.getConfigurator().apply(output);

        m_follower.getConfigurator().apply(config);
        m_follower.setControl(new Follower(m_leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // ---------------------------------------------------------------- control

    private void setVelocity(double rps) {
        m_targetVelocity = rps;
        targetVelocityEntry.setDouble(rps);
        m_leader.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void startRoller() {
        setVelocity(k_defaultVelocity);
    }
    
    public void stopRoller() {
        setVelocity(0);
    }

    public void reverseRoller() {
        setVelocity(-k_defaultVelocity);
    }

    public void toggle_roller() {
        m_rollerOn = !m_rollerOn;
        if (m_rollerOn) {
            startRoller();
        } else {
            stopRoller();
        }
    }

    public boolean roller_toggled() {
        return m_rollerOn;
    }

    public boolean roller_not_toggled() {
        return !m_rollerOn;
    }

    // --------------------------------------------------------------- commands

    public Command delay_stopRoller_Command(double delay) {
        return new WaitCommand(delay).andThen(new InstantCommand(this::stopRoller, this));
    }

    public Command auto_intake_fuel_command() {
        return new InstantCommand(this::startRoller, this);
    }

    public Command auto_intake_fuel_stop() {
        return new InstantCommand(this::stopRoller, this);
    }

    public Command auto_outtake_fuel_command() {
        return run(() -> setVelocity(k_outtakeVelocity))
            .withTimeout(1)
            .andThen(this::stopRoller);
    }

    // --------------------------------------------------------------- telemetry

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Set Point", m_leader.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Intake Error", m_leader.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("Intake Torque Current", m_leader.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Velocity", m_leader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Acceleration", m_leader.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber("Intake Position", m_leader.getPosition().getValueAsDouble());

        currentVelocityEntry.setDouble(m_leader.getVelocity().getValueAsDouble());
        temperatureEntry.setDouble(m_leader.getDeviceTemp().getValueAsDouble());
    }
}