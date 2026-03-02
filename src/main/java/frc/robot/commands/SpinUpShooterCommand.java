package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Spins the shooter motors up to speed.
 * Use this as a named command in PathPlanner to rev the shooter
 * while the robot is still driving to a scoring position.
 */
public class SpinUpShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;

    public SpinUpShooterCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_shooter.runForward();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
