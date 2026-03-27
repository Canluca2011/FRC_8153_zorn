// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.commands.AlignWithAprilTagCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUpShooterCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

public class RobotContainer {
    // Auto chooser for selecting autonomous routines
    private final SendableChooser<Command> autoChooser;
    // Maximum speeds obtained from TunerConstants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //(m/s)
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); //(rad/s)

    /* Swerve drive request for field-centric control */ // PID AUTOCENTER MIGHT BREAK!!!!!!!!!!!!!!!!
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveConstants.kDeadband)
            .withRotationalDeadband(MaxAngularRate * DriveConstants.kDeadband) //10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controller for driver inp
    public final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final TransportSubsystem transport = new TransportSubsystem();

    public RobotContainer() {
        // Register named commands BEFORE building auto chooser
        NamedCommands.registerCommand("spinUp", new SpinUpShooterCommand(shooter));
        NamedCommands.registerCommand("shoot", new ShootCommand(shooter, transport));

        configureBindings();

        // Build an auto chooser with all autos in the project
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Create the AprilTag alignment command
        AlignWithAprilTagCommand alignWithAprilTagCommand = 
            new AlignWithAprilTagCommand(drivetrain, drive, joystick);
            
        // Add a description to the SmartDashboard
        SmartDashboard.putString("AprilTag/Info", "Press POV Up to align with AprilTag");

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -MathUtil.applyDeadband(joystick.getLeftY(), DriveConstants.kDeadband) * MaxSpeed;
                double velocityY = -MathUtil.applyDeadband(joystick.getLeftX(), DriveConstants.kDeadband) * MaxSpeed;
                
                double rotationalRate = -MathUtil.applyDeadband(joystick.getRightX(), DriveConstants.kDeadband) * MaxAngularRate;
            
                return drive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
            })
        );
        
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // Reset Field-Centric Reference
        
        // Bind the AprilTag alignment command to POV Up
        joystick.povUp().whileTrue(alignWithAprilTagCommand);


        drivetrain.registerTelemetry(logger::telemeterize);
        
        //Subsystem Buttons
        joystick.x().toggleOnTrue(Commands.startEnd(intake::runForward, intake::stop, intake)); //Intake Toggle
        joystick.a().whileTrue(intake.runEnd(intake::runReverse, intake::stop)); //Intake Reverse
        joystick.y().whileTrue(shooter.runEnd(shooter::runForward, shooter::stop)); //Shooter Shoot
        joystick.rightBumper().whileTrue(transport.runEnd(transport::runForward, transport::stop)); //Transport Forward
        joystick.leftBumper().whileTrue(transport.runEnd(transport::runReverse, transport::stop)); //Transport Reverse
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
