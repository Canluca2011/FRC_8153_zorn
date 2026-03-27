// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/**
 * A command that aligns the robot's heading to center an AprilTag using Limelight's tx.
 * It uses trigonometry (ty) to calculate distance and stops aligning when close enough.
 * The driver can still control the robot's movement while the command handles the rotation.
 */
public class AlignWithAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final FieldCentric m_drive; 
  private final CommandXboxController m_joystick; 
  private final PIDController m_rotationPID;

  // ==========================================
  // HARDCODED CONFIGURATION VARIABLES
  // ==========================================
  private final String m_limelightName = "limelight-emperor"; // Default limelight network table name
  private final double m_closeEnoughDistance = 0.75; // Distance in meters to stop aligning
  private final double m_maxSpeed = 4.5; // Max translation speed (meters per second)
  private final double m_maxAngularRate = Math.PI; // Max rotation speed (radians per second)
  private static double distance = 0.0; // Distance to target (meters) - for debugging

  // Trigonometry Constants for Distance Calculation
  // NOTE: UPDATE THESE TO MATCH YOUR ACTUAL ROBOT MEASUREMENTS
  private final double m_cameraHeightMeters = 0.47; // Height of the Limelight lens from the floor
  private final double m_targetHeightMeters = 1.1; // Height of the AprilTag center from the floor
  private final double m_cameraPitchDegrees = 30.0; // Mounting angle of the camera (up is positive)

  // PID constants for rotation control (pulling from your Constants file)
  private static final double kP = VisionConstants.kAlignmentP;
  private static final double kI = VisionConstants.kAlignmentI;
  private static final double kD = VisionConstants.kAlignmentD;
  private static final double TOLERANCE = VisionConstants.kAlignmentTolerance; // degrees

  /**
   * Creates a new AlignWithAprilTagCommand with hardcoded constraints.
   * * @param drivetrain The drivetrain subsystem to use
   * @param drive The FieldCentric drive request to use
   * @param joystick The controller for driver input
   */
  public AlignWithAprilTagCommand(CommandSwerveDrivetrain drivetrain, FieldCentric drive, CommandXboxController joystick) {
    m_drivetrain = drivetrain;
    m_drive = drive;
    m_joystick = joystick;
    
    m_rotationPID = new PIDController(kP, kI, kD);
    m_rotationPID.setTolerance(TOLERANCE);
    m_rotationPID.enableContinuousInput(-180, 180); 
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_rotationPID.reset();
  }

  @Override
  public void execute() {
    // Get driver input for movement (with deadband matching default command)
    double velocityX = -MathUtil.applyDeadband(m_joystick.getLeftY(), DriveConstants.kDeadband) * m_maxSpeed;
    double velocityY = -MathUtil.applyDeadband(m_joystick.getLeftX(), DriveConstants.kDeadband) * m_maxSpeed;
    
    // Check if we have a valid target
    boolean hasValidTarget = LimelightHelpers.getTV(m_limelightName);
    SmartDashboard.putBoolean("AprilTag/HasTarget", hasValidTarget);
    
    if (!hasValidTarget) {
      // No valid target, allow manual rotation
      double rotationalRate = -MathUtil.applyDeadband(m_joystick.getRightX(), DriveConstants.kDeadband) * m_maxAngularRate;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Get the horizontal (tx) and vertical (ty) offsets from the Limelight
    double tx = LimelightHelpers.getTX(m_limelightName);
    double ty = LimelightHelpers.getTY(m_limelightName);

    // Calculate distance to target using trigonometry
    double angleToGoalRadians = Math.toRadians(m_cameraPitchDegrees + ty);
    double distance = (m_targetHeightMeters - m_cameraHeightMeters) / Math.tan(angleToGoalRadians);
    
    SmartDashboard.putNumber("AprilTag/Distance", distance);
    AlignWithAprilTagCommand.distance = distance;

    // If we're close enough to the tag, we can stop aligning
    boolean isCloseEnough = distance < m_closeEnoughDistance;
    SmartDashboard.putBoolean("AprilTag/IsCloseEnough", isCloseEnough);
    
    if (isCloseEnough) {
      // Close enough, allow manual rotation
      double rotationalRate = -MathUtil.applyDeadband(m_joystick.getRightX(), DriveConstants.kDeadband) * m_maxAngularRate;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(-rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Use PID to calculate the rotational rate. We want to bring tx to 0.
    double rotationRate = m_rotationPID.calculate(tx, 0);
    
    // Clamp the rotation rate so the robot doesn't spin too violently
    rotationRate = MathUtil.clamp(rotationRate, -m_maxAngularRate, m_maxAngularRate);

    SmartDashboard.putNumber("AprilTag/TX", tx);
    SmartDashboard.putNumber("AprilTag/RotationRate", rotationRate);
    
    // Apply the rotation to the drivetrain while allowing driver control of translation
    m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(-rotationRate));
    SmartDashboard.putBoolean("AprilTag/IsAligning", true);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AprilTag/IsAligning", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}