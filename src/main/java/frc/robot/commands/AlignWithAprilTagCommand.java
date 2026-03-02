// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.VisionConstants;

/**
 * A command that aligns the robot's heading to be parallel with the plane of an AprilTag.
 * This makes the robot face directly towards the tag, which helps with alignment.
 * The driver can still control the robot's movement while the command handles the rotation.
 */
public class AlignWithAprilTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final String m_limelightName;
  private final PIDController m_rotationPID;
  private final double m_closeEnoughDistance; // Distance in meters to consider "close enough" to the tag
  private final FieldCentric m_drive; // Swerve request for field-centric control
  private final CommandXboxController m_joystick; // Controller for driver input
  private final double m_maxSpeed; // Maximum speed in meters per second

  

// ...existing code...
  // PID constants for rotation control
  private static final double kP = VisionConstants.kAlignmentP;
  private static final double kI = VisionConstants.kAlignmentI;
  private static final double kD = VisionConstants.kAlignmentD;
  private static final double TOLERANCE = VisionConstants.kAlignmentTolerance; // degrees

  /**
   * Creates a new AlignWithAprilTagCommand.
   * 
   * @param drivetrain The drivetrain subsystem to use
   * @param limelightName The name of the Limelight camera
   * @param closeEnoughDistance Distance in meters to consider "close enough" to the tag
   * @param drive The FieldCentric drive request to use
   * @param joystick The controller for driver input
   * @param maxSpeed The maximum speed in meters per second
   */
  public AlignWithAprilTagCommand(CommandSwerveDrivetrain drivetrain, String limelightName, 
                                 double closeEnoughDistance, FieldCentric drive,
                                 CommandXboxController joystick, double maxSpeed) {
    m_drivetrain = drivetrain;
    m_limelightName = limelightName;
    m_closeEnoughDistance = closeEnoughDistance;
    m_drive = drive;
    m_joystick = joystick;
    m_maxSpeed = maxSpeed;
    
    m_rotationPID = new PIDController(kP, kI, kD);
    m_rotationPID.setTolerance(TOLERANCE);
    m_rotationPID.enableContinuousInput(-180, 180); // Treat angle wrapping properly
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_rotationPID.reset();
  }

  @Override
  public void execute() {
    /* 
     * RECOMMENDATIONS FOR TOURNAMENT PREP:
     * 1. Coordinate System Verification:
     *    - Test if pushing the joystick LEFT moves the robot LEFT. If it moves RIGHT, check velocityY.
     *    - Test if pushing UP moves the robot FORWARD.
     * 
     * 2. PID Tuning:
     *    - Current P=0.1 might be too soft. If the robot aligns too slowly, increase P (try 0.15 or 0.2).
     *    - If the robot oscillates (wobbles left/right), increase D (try 0.02).
     * 
     * 3. "Close Enough" Behavior:
     *    - Currently, if the robot gets within m_closeEnoughDistance, alignment STOPS.
     *    - If you are missing shots because the robot stops aligning too early, consider removing the "isCloseEnough" check
     *      so it continues to align even while approaching the tag.
     */

    // Get driver input for movement
    double velocityX = -m_joystick.getLeftY() * m_maxSpeed;
    double velocityY = m_joystick.getLeftX() * m_maxSpeed;
    
    // Check if we have a valid target
    boolean hasValidTarget = LimelightHelpers.getTV(m_limelightName);
    SmartDashboard.putBoolean("AprilTag/HasTarget", hasValidTarget);
    SmartDashboard.putString("AprilTag/LimelightName", m_limelightName);
    
    if (!hasValidTarget) {
      // No valid target, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Get the target's pose in robot space
    double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(m_limelightName);
    
    if (targetPose.length < 6) {
      // Invalid pose data, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Calculate distance to target (using x and y components)
    double distance = Math.sqrt(targetPose[0] * targetPose[0] + targetPose[1] * targetPose[1]);
    SmartDashboard.putNumber("AprilTag/Distance", distance);
    
    // If we're close enough to the tag, we can stop aligning
    boolean isCloseEnough = distance < m_closeEnoughDistance;
    SmartDashboard.putBoolean("AprilTag/IsCloseEnough", isCloseEnough);
    
    if (isCloseEnough) {
      // Close enough, allow manual rotation
      double rotationalRate = -m_joystick.getRightX() * m_maxSpeed;
      m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationalRate));
      SmartDashboard.putBoolean("AprilTag/IsAligning", false);
      return;
    }

    // Extract the yaw component from the target pose
    // The yaw represents the rotation around the Z axis
    double targetYaw = targetPose[5]; // In degrees
    
    // Calculate the heading error
    // We want to align with the tag, so our desired heading is 0 degrees relative to the tag
    // The error is the negative of the target's yaw in robot space
    double headingError = -targetYaw;
    SmartDashboard.putNumber("AprilTag/HeadingError", headingError);
    
    // Use PID to calculate the rotational rate
    double rotationRate = m_rotationPID.calculate(headingError, 0);
    SmartDashboard.putNumber("AprilTag/RotationRate", rotationRate);
    
    // Apply the rotation to the drivetrain while allowing driver control of movement
    m_drivetrain.setControl(m_drive.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(rotationRate));
    SmartDashboard.putBoolean("AprilTag/IsAligning", true);
  }

  @Override
  public void end(boolean interrupted) {
    // Let the default command take over when this command ends
    SmartDashboard.putBoolean("AprilTag/IsAligning", false);
  }

  @Override
  public boolean isFinished() {
    // This command runs until interrupted
    return false;
  }
}
