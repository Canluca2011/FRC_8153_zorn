// RobotContainer.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.commands.AlignWithAprilTagCommand;
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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controller for driver inp
    public final CommandXboxController joystick = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final TransportSubsystem transport = new TransportSubsystem();

    public RobotContainer() {
        configureBindings();
        
        // Build an auto chooser with all autos in the project
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Create the AprilTag alignment command
        AlignWithAprilTagCommand alignWithAprilTagCommand = 
            new AlignWithAprilTagCommand(drivetrain, VisionConstants.kLimelightName, VisionConstants.kCloseEnoughDistanceMin, drive, joystick, MaxSpeed);
            
        // Add a description to the SmartDashboard
        SmartDashboard.putString("AprilTag/Info", "Press POV Up to align with AprilTag");

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -MathUtil.applyDeadband(joystick.getLeftY(), DriveConstants.kDeadband) * MaxSpeed;
                double velocityY = -MathUtil.applyDeadband(joystick.getLeftX(), DriveConstants.kDeadband) * MaxSpeed;
                
                double rotationalRate = -MathUtil.applyDeadband(joystick.getRightX(), DriveConstants.kDeadband) * MaxAngularRate;
                
                // The AprilTag alignment is now handled by the AlignWithAprilTagCommand
                // when button 3 is pressed, so we don't need to check for it here
                
                if(joystick.button(8).getAsBoolean()){
                }
                else if(joystick.button(9).getAsBoolean()){

                }
                else{

                }
            
                return drive.withVelocityX(velocityX)
                            .withVelocityY(velocityY)
                            .withRotationalRate(rotationalRate);
            })
        );
        
        // Bind the AprilTag alignment command to button 3
        // DISABLED - AprilTag alignment temporarily deactivated
        // joystick.button(3).whileTrue(alignWithAprilTagCommand);
        
        /* 
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        */

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        joystick.povDown().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())); // Reset Field-Centric Reference
        
        // Bind the AprilTag alignment command to POV Up
        joystick.povUp().whileTrue(alignWithAprilTagCommand);


        drivetrain.registerTelemetry(logger::telemeterize);
        
        //Subsystem Buttons
        joystick.x().whileTrue(intake.runEnd(intake::runForward, intake::stop)); //Intake Forward
        joystick.a().whileTrue(intake.runEnd(intake::runReverse, intake::stop)); //Intake Reverse
        joystick.y().whileTrue(shooter.runEnd(shooter::runForward, shooter::stop)); //Shooter Shoot
        joystick.rightBumper().whileTrue(transport.runEnd(transport::runForward, transport::stop)); //Transport Forward
        joystick.leftBumper().whileTrue(transport.runEnd(transport::runReverse, transport::stop)); //Transport Reverse
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        return autoChooser.getSelected();
        
        // Alternatively, you can return a specific auto:
        // return new PathPlannerAuto("Example Auto");
    }
}
