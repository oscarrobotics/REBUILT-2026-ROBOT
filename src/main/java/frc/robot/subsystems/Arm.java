package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Intake arm: one TalonFX (52) doing MotionMagic position control. Position 0 = retracted. */
public class Arm extends SubsystemBase {

    private static final int k_motorId = 52;

    // Arm travel (motor rotations).
    private final Angle arm_start = Rotations.of(1.39);
    private final Angle arm_end = Rotations.of(56); // 57.917-3
    private final Angle arm_delta = arm_end.minus(arm_start);

    private final TalonFX m_motor = new TalonFX(k_motorId);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Intake Arm");
    private final GenericEntry armRotationEntry = tab.add("Arm Rotation", 0).getEntry();
    private final GenericEntry temperatureEntry = tab.add("Arm Motor Temperature (C)", 0).getEntry();

    private boolean m_armOut = false;

    public Arm() {
        m_motor.setPosition(0);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // NOTE: in the old combined Intake class these were read from the *roller's* Shuffleboard
        // entries (which already existed, so the 0.41 / 0.22 / ... fallback values were never used).
        // The gains below are what the arm was actually running with. The values you originally
        // typed were: kV 0.22, kA 0.02, kP 0.41, kI 0.001, kD 0.004. Try those if you want them.
        config.Slot0.kV = 0.12;
        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.11;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.CurrentLimits.StatorCurrentLimit = 30;

        config.MotionMagic.MotionMagicCruiseVelocity = 80;
        config.MotionMagic.MotionMagicAcceleration = 200;
        config.MotionMagic.MotionMagicJerk = 2000;

        m_motor.getConfigurator().apply(config);

        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = NeutralModeValue.Coast;
        output.DutyCycleNeutralDeadband = 0.02;
        m_motor.getConfigurator().apply(output);
    }

    // ---------------------------------------------------------------- control

    public void extendArm() {
        m_armOut = true;
        m_motor.setControl(m_positionRequest.withPosition(arm_delta));
    }

    public void retractArm() {
        m_armOut = false;
        m_motor.setControl(m_positionRequest.withPosition(0));
    }

    public void toggle_arm() {
        if (m_armOut) {
            retractArm();
        } else {
            extendArm();
        }
    }

    public boolean isDeployed() {
        return m_armOut;
    }

    public void half_deploy() {
        m_motor.setControl(m_positionRequest.withPosition(arm_delta.times(0.5)));
    }

    /** Joystick-driven wiggle around the fully-out position. Only acts while the arm is deployed. */
    public void active_wiggle(double wiggle) {
        Angle out_wiggle = Rotations.of(57).minus(arm_delta);
        Angle in_wiggle = Rotations.of(32);

        Angle adjust = wiggle < 0 ? out_wiggle.times(wiggle) : in_wiggle.times(wiggle);

        if (m_armOut) {
            m_motor.setControl(m_positionRequest.withPosition(arm_delta.minus(adjust)));
        }
    }

    /** Tells the arm its sensor reads "fully out" (use after a reset with the arm already out). */
    public void set_position_as_out() {
        m_motor.setPosition(arm_delta.in(Rotations));
    }

    // --------------------------------------------------------------- commands

    /**
     * Slowly pulls the arm in from the fully-out position to "squish" the fuel.
     * Runs until cancelled; afterward the default command (active_wiggle) takes the arm back out.
     */
    public Command squish_arm_command(double rotationsIn, double seconds) {
        Timer timer = new Timer();
        return new FunctionalCommand(
            timer::restart,
            () -> {
                double fraction = Math.min(timer.get() / seconds, 1.0);
                double target = arm_delta.in(Rotations) - rotationsIn * fraction;
                m_motor.setControl(m_positionRequest.withPosition(target));
            },
            interrupted -> {},
            () -> false,
            this
        ).onlyIf(this::isDeployed);
    }

    public ParallelRaceGroup auto_wiggle() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> m_motor.setControl(m_positionRequest.withPosition(arm_delta.div(1.5))), this),
            new WaitCommand(0.8),
            new InstantCommand(() -> m_motor.setControl(m_positionRequest.withPosition(arm_delta)), this),
            new WaitCommand(0.8)
        ).repeatedly().withTimeout(5);
    }

    public Command auto_extract_out_intake_command() {
        return Commands.sequence(
            new InstantCommand(this::extendArm, this),
            new WaitCommand(1)
        );
    }

    public Command auto_retract_in_intake_command() {
        return Commands.sequence(
            new InstantCommand(this::retractArm, this),
            new WaitCommand(1)
        );
    }

    // --------------------------------------------------------------- telemetry

    @Override
    public void periodic() {
        armRotationEntry.setDouble(m_motor.getPosition().getValueAsDouble());
        temperatureEntry.setDouble(m_motor.getDeviceTemp().getValueAsDouble());
    }
}