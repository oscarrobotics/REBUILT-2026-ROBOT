// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    // Shooter spins immediately, hopper+feeder start at 0.3 s, arm starts squishing at 0.75 s.
private Command shootSequence(Supplier<AngularVelocity> speed) {
    return Commands.parallel(
        shooter.runEnd(() -> shooter.StartShooter(speed.get()), shooter::StopShooter),

        Commands.waitSeconds(1).andThen( //time shooter windup
            Commands.startEnd(
                () -> { hopper.startHopper(); feeder.startFeeder(); intakeRoller.startRoller();},
                () -> { hopper.stopHopper();  feeder.stopFeeder(); intakeRoller.stopRoller();},
                hopper, feeder, intakeRoller)),//check if this works Gino!

        Commands.waitSeconds(0.75).andThen(
            arm.auto_wiggle())
              // 15 rotations in, over 1.5 seconds
            //Youre going to the trashcan if you dont work
    );
}


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle locked_drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            // .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric testdrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed* 0.1).withRotationalDeadband(MaxAngularRate*0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            // .withDriveRequestType(DriveRequestType.Velocity);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivestick = new CommandXboxController(0);
    private final CommandXboxController operatorstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter(drivetrain);
    public final Arm arm = new Arm();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final Feeder feeder =   new Feeder(TunerConstants.kCANBus,drivetrain);
   // public final Climber climber = new Climber();
    public final Hopper hopper =   new Hopper(drivetrain);

    public final Vision vision = new Vision(drivetrain);

    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
       
        configureSystemsBindings();
        name_commands();
        drivetrain.configure_autobuilder();
        autoChooser = AutoBuilder.buildAutoChooser("trench");
        SmartDashboard.putData("Auto Mode", autoChooser);
        Shuffleboard.getTab("autonomous path").add(autoChooser);

        configureBindings();

        configureAutoBindings();

    }

      public void periodic() {
        vision.megaTagPose_periodic();
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        //field centric drive, use by default:
         
        

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed*1) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivestick.getLeftX() * MaxSpeed*1) // Drive left with negativeX (left)
                    .withRotationalRate(-drivestick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    
                    )
        );

        // drivestick.a().toggleOnTrue(drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(drivetrain.get_rate_to_lock()) // Drive counterclockwise with negative X (left)
                    
        //             )
        // );
        drivestick.b().toggleOnTrue(drivetrain.applyRequest(() ->
                locked_drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withHeadingPID(5,0, 0.01)
                    .withTargetDirection(new Rotation2d(drivetrain.get_target_angle().plus(Degree.of(7).times(drivestick.getRightX()))))
                    
                    )
        );
        
        // drivestick.y().whileTrue(new DeferredCommand(drivetrain::navigate_2_nearest_trench,  Set.of(drivetrain)));

        // drivestick.x().toggleOnTrue(drivetrain.applyRequest(() ->
        //         locked_drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withHeadingPID(5,0, 0.01)
        //             .withTargetDirection(new Rotation2d(drivetrain.get_target_moving_angle()))
                    
        //             )
        // );

        drivestick.povUp().and(arm::isDeployed)
        .whileTrue(new InstantCommand(intakeRoller::startRoller, intakeRoller));
        
        drivestick.povUp().and(arm::isDeployed).and(intakeRoller::roller_not_toggled)
        .onFalse(intakeRoller.delay_stopRoller_Command(0.5));



        drivestick.povUp().whileTrue(drivetrain.applyRequest(()->
            testdrive.withVelocityX(MaxSpeed*0.5)            
            .withRotationalRate(-drivestick.getRightX() * MaxAngularRate)
            )).onFalse(drivetrain.applyRequest(()->testdrive.withVelocityX(0)).withTimeout(0.02));
        
        drivestick.povDown().whileTrue(drivetrain.applyRequest(()->
            testdrive.withVelocityX(-MaxSpeed*0.5)            
            .withRotationalRate(-drivestick.getRightX() * MaxAngularRate)
            )).onFalse(drivetrain.applyRequest(()->testdrive.withVelocityX(0)).withTimeout(0.02));

        drivestick.povLeft().whileTrue(drivetrain.applyRequest(()->
            testdrive.withVelocityY(MaxSpeed*0.5)
            .withRotationalRate(-drivestick.getRightX() * MaxAngularRate)
            )).onFalse(drivetrain.applyRequest(()->testdrive.withVelocityY(0)).withTimeout(0.02));

        drivestick.povRight().whileTrue(drivetrain.applyRequest(()->
            testdrive.withVelocityY(-MaxSpeed*0.5)          
            .withRotationalRate(-drivestick.getRightX() * MaxAngularRate)
            )).onFalse(drivetrain.applyRequest(()->testdrive.withVelocityY(0)).withTimeout(0.02));

            // Drivetrain will execute this command periodically
            
        
        

        // robot  centric drive, use for testing and in special situations:
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         testdrive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-drivestick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // // drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // drivestick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivestick.rightBumper().onTrue(new InstantCommand(intakeRoller::toggle_roller, intakeRoller));//changed from toggle because pulley rubbing against wall and we can't put a spacer
        drivestick.rightTrigger().onTrue(new InstantCommand(intakeRoller::reverseRoller, intakeRoller)).onFalse(new InstantCommand(intakeRoller::stopRoller, intakeRoller));

        // Reset the field-centric heading on left bumper press.
        drivestick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivestick.start().onTrue(drivetrain.runOnce(vision::seedFieldCentricFromCamera));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // where subsystem controls will be 
    public void configureSystemsBindings() {
        // Configure additional subsystems and bindings here.

// Left trigger: driver manual aim. Preset speed from the D-pad, left stick Y trims ±800 RPM.
operatorstick.leftTrigger().whileTrue(shootSequence(() ->
    shooter.speed_setpoint.plus(
        RPM.of(800).times(-MathUtil.applyDeadband(operatorstick.getRightY(), 0.1)))));

// Right trigger: auto aim. Distance-based speed, left stick Y trims ±800 RPM.
/*eratorstick.rightTrigger().whileTrue(shootSequence(() ->
    shooter.get_auto_speed().plus(
        RPM.of(800).times(-MathUtil.applyDeadband(operatorstick.getLeftY(), 0.1)))));*/
//PLEASE IF THERE IS A GOD, WORK!

//Ian's shooter code, probably better(?)

operatorstick.rightTrigger().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.StartShooter(shooter.get_auto_speed().plus(RPM.of(800).times(-operatorstick.getLeftY()))), shooter))
        ).onFalse(shooter.stopCommand());
        
        operatorstick.rightTrigger().and(shooter::shooteratSpeed).and(shooter::shooterAimed).and(()->!operatorstick.leftBumper().getAsBoolean())
        .whileTrue(new RepeatCommand(new InstantCommand(hopper::startHopper)))
        .whileTrue(new RepeatCommand(new InstantCommand(feeder::startFeeder)))
        .onFalse(new InstantCommand(hopper::stopHopper)).onFalse(new InstantCommand(feeder::stopFeeder));
 

        operatorstick.povUp().onTrue(new InstantCommand(()->{shooter.speed_setpoint=shooter.close_speed;})); //near hub
        operatorstick.povDown().onTrue(new InstantCommand(()->{shooter.speed_setpoint=shooter.full_speed;})); //farthest away
        operatorstick.povLeft().onTrue(new InstantCommand(()->{shooter.speed_setpoint=shooter.far_speed;})); //mid-far way
        operatorstick.povRight().onTrue(new InstantCommand(()->{shooter.speed_setpoint=shooter.opt_speed;})); //mid-far way
   

        

        //Feeder Controls
        operatorstick.b().onTrue(new InstantCommand(feeder::startFeeder, feeder)).onFalse(new InstantCommand(feeder::stopFeeder, feeder));
        
        //Hopper Controls
        operatorstick.rightBumper().whileTrue(new RepeatCommand( new InstantCommand(hopper::startHopper, hopper))).onFalse(new InstantCommand(hopper::stopHopper, hopper));
        operatorstick.leftBumper().onTrue(new InstantCommand(hopper::reverseHopper, hopper)).onFalse(new InstantCommand(hopper::stopHopper, hopper));
        

        //drops intake 
        operatorstick.y().onTrue(new InstantCommand(arm::toggle_arm, arm));
        operatorstick.x().onTrue(new InstantCommand(intakeRoller::reverseRoller, intakeRoller)).onFalse(new InstantCommand(intakeRoller::stopRoller, intakeRoller));

        // operatorstick.leftTrigger().onTrue(new InstantCommand(intakeRoller::toggle_roller, intakeRoller));


    operatorstick.start().onTrue(new InstantCommand(arm::set_position_as_out, arm));
//sets the sensor position as if the arm were out, i.e if the robot was reset
//with the arm out. This fixes the sensor.


    arm.setDefaultCommand(new InstantCommand(()->arm.active_wiggle(operatorstick.getLeftY()), arm));
        





    }

    private void configureAutoBindings(){

//event trigger
        new EventTrigger("autointakefuel_trigger").onTrue(intakeRoller.auto_intake_fuel_command());
        new EventTrigger("autointakefuelstop_trigger").onTrue(intakeRoller.auto_intake_fuel_stop());
        new EventTrigger("extendintakearm_trigger").onTrue(new InstantCommand(arm::extendArm));
        new EventTrigger("retractintakearm_trigger").onTrue(new InstantCommand(arm::retractArm));
    }
            
     
    private void name_commands() {
        
        //commands
        NamedCommands.registerCommand("autointakefuel", intakeRoller.auto_intake_fuel_command());
        NamedCommands.registerCommand("autointakefuelstop", intakeRoller.auto_intake_fuel_stop());
        NamedCommands.registerCommand("autointakearmhalf", new InstantCommand(arm::half_deploy));
        NamedCommands.registerCommand("extendintakearm", new InstantCommand(arm::extendArm));
        NamedCommands.registerCommand("retractintakearm", new InstantCommand(arm::retractArm));
        
        NamedCommands.registerCommand("autoshoot", shooter.autoshoot());
        NamedCommands.registerCommand("autoshootstop", new InstantCommand(shooter::StopShooter, shooter));

        NamedCommands.registerCommand("autofeederstart", feeder.auto_feeder_start());
        NamedCommands.registerCommand("autofeederstop", feeder.auto_feeder_end());

        NamedCommands.registerCommand("autohopperstart", hopper.auto_start_hopper());
        NamedCommands.registerCommand("autohopperstop", hopper.auto_stop_hopper());

        NamedCommands.registerCommand("armwiggle", arm.auto_wiggle());
        NamedCommands.registerCommand("auto_aim", drivetrain.instant_autoAim(locked_drive));
       
    }




    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        
        return autoChooser.getSelected();
    }
}