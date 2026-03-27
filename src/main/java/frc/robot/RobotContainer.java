// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    public final Intake intake =   new Intake(drivetrain);
    public final Feeder feeder =   new Feeder(TunerConstants.kCANBus,drivetrain);
    public final Climber climber = new Climber();
    public final Hopper hopper =   new Hopper(drivetrain);

    public final Vision vision = new Vision(drivetrain);

    
    private final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        configureBindings();
        configureSystemsBindings();
        name_commands();
        drivetrain.configure_autobuilder();
        autoChooser = AutoBuilder.buildAutoChooser("auto paths");
        Shuffleboard.getTab("autonomouspath").add(autoChooser);

        

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

        // drivestick.x().toggleOnTrue(drivetrain.applyRequest(() ->
        //         locked_drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withHeadingPID(5,0, 0.01)
        //             .withTargetDirection(new Rotation2d(drivetrain.get_target_moving_angle()))
                    
        //             )
        // );

        drivestick.povUp().and(intake::isDeployed)
        .whileTrue(new InstantCommand(intake::startRoller, intake));
        
        drivestick.povUp().and(intake::isDeployed).and(intake::roller_not_toggled)
        .onFalse(intake.delay_stopRoller_Command(0.5));



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

        // Reset the field-centric heading on left bumper press.
        drivestick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // where subsystem controls will be 
    public void configureSystemsBindings() {
        // Configure additional subsystems and bindings here.

          //shooter command pressing right trigger - use joystick to adjust (min-optimatal-max)
    
        // operatorstick.rightTrigger().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.StartShooter(shooter.speed_setpoint.plus(RPM.of(800).times(-operatorstick.getLeftY()))), shooter))
        // ).onFalse(shooter.stopCommand());




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
        operatorstick.rightBumper().whileTrue(new InstantCommand(feeder::startFeeder, feeder)).onFalse(new InstantCommand(feeder::stopFeeder, feeder));
        
        //Hopper Controls
        operatorstick.rightBumper().whileTrue(new RepeatCommand( new InstantCommand(hopper::startHopper, hopper))).onFalse(new InstantCommand(hopper::stopHopper, hopper));
        operatorstick.leftBumper().onTrue(new InstantCommand(hopper::reverseHopper, hopper)).onFalse(new InstantCommand(hopper::stopHopper, hopper));
        

        //drops intake 
        operatorstick.y().onTrue(new InstantCommand(intake::toggle_arm, intake));
        operatorstick.x().onTrue(new InstantCommand(intake::reverseRoller, intake)).onFalse(new InstantCommand(intake::stopRoller));

        operatorstick.leftTrigger().onTrue(new InstantCommand(intake::toggle_roller, intake));


        operatorstick.start().onTrue(new InstantCommand(intake::set_position_as_out, intake));



        intake.setDefaultCommand(new InstantCommand(()->intake.active_wiggle(operatorstick.getRightY()), intake));
        
    }
            
     
    private void name_commands() {
        NamedCommands.registerCommand("autointakefuel", intake.auto_intake_fuel_command());
        NamedCommands.registerCommand("autointakefuelstop", intake.auto_intake_fuel_stop());
        NamedCommands.registerCommand("autointakearmhalf", new InstantCommand(intake::half_deploy));
        
        NamedCommands.registerCommand("autoshoot", shooter.autoshoot());
        NamedCommands.registerCommand("autoshootstop", new InstantCommand(shooter::StopShooter, shooter));

        NamedCommands.registerCommand("autofeederstart", feeder.auto_feeder_start());
        NamedCommands.registerCommand("autofeederstop", feeder.auto_feeder_end());

        NamedCommands.registerCommand("autohopperstart", hopper.auto_start_hopper());
        NamedCommands.registerCommand("autohopperstop", hopper.auto_stop_hopper());
       
    }



    public Command getAutonomousCommand() {

        // return autoChooser.getSelected();
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return autoChooser.getSelected();
       return Commands.sequence(
            // (near the hub) 
            // extract intake 
            // wait 0.5 second
            // set shooter speed 1000
            

            // intake.auto_extract_out_intake_command(),
            new InstantCommand(intake::half_deploy), new WaitCommand(0.5),
            // drivetrain.applyRequest(() ->
            //     locked_drive
            //         // .withVelocityX(-drivestick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            //         // .withVelocityY(-drivestick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            //         .withHeadingPID(5,0, 0.01)
            //         .withTargetDirection(new Rotation2d(drivetrain.get_target_angle()))
                    
            // ),
            
            shooter.autoshoot(),
            feeder.auto_feeder_start(),
            hopper.auto_start_hopper(),
            new WaitCommand(3), 
            intake.auto_intake_fuel_command(),  
            new WaitCommand(5),
            new InstantCommand(shooter::StopShooter, shooter),
            feeder.auto_feeder_end(),
            hopper.auto_stop_hopper(),
            intake.auto_intake_fuel_stop()
        
       );


    }

}


