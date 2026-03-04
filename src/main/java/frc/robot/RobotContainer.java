// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric testdrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed* 0.1).withRotationalDeadband(MaxAngularRate*0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivestick = new CommandXboxController(0);
    private final CommandXboxController operatorstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter(drivetrain);
    public final Intake intake = new Intake();
    public final Feeder feeder = new Feeder(TunerConstants.kCANBus);
    public final Climber climber = new Climber();
    public final Hopper hopper = new Hopper();

    public final Vision vision = new Vision(drivetrain);
    
    public RobotContainer() {
        configureBindings();
        configureSystemsBindings();
        name_commands();

    }

      public void periodic() {
        vision.megaTagPose_periodic();
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        //field centric drive, use by default:
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // robot  centric drive, use for testing and in special situations:
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                testdrive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-drivestick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        drivestick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        drivestick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // where subsystem controls will be 
    public void configureSystemsBindings() {
        // Configure additional subsystems and bindings here.

          //shooter command pressing right trigger - use joystick to adjust (min-optimatal-max)
        operatorstick.rightTrigger().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.StartShooter(shooter.opt_speed.plus(RPM.of(800).times(-operatorstick.getLeftY()))), shooter))
        ).onFalse(shooter.stopCommand());
        

        //starting feeeder 
        operatorstick.rightBumper().whileTrue(new RepeatCommand( new InstantCommand(feeder::startFeeder, feeder))).onFalse(new InstantCommand(feeder::stopFeeder, feeder));
        operatorstick.rightBumper().whileTrue(new RepeatCommand( new InstantCommand(hopper::startHopper, hopper))).onFalse(new InstantCommand(hopper::stopHopper, hopper));

        //drops intake 
        operatorstick.y().onTrue(new InstantCommand(intake::toggle_arm, intake));

        operatorstick.leftTrigger().onTrue(new InstantCommand(intake::toggle_roller, intake));


        intake.setDefaultCommand(new InstantCommand(()->intake.active_wiggle(operatorstick.getRightY())));
        
    }
            
        
    public void name_commands(){
        NamedCommands.registerCommand("autoshoot", 

    }


    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );

    }

}


