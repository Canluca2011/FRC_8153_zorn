package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

/**
 * Spins the shooter AND runs the transport to feed and fire a game piece.
 * Use this as a named command in PathPlanner to score during autonomous.
 * Automatically ends after a set duration.
 */
public class ShootCommand extends Command {
    private static final double kShootDurationSeconds = 6.0;

    private final ShooterSubsystem m_shooter;
    private final TransportSubsystem m_transport;
    private final Timer m_timer = new Timer();

    public ShootCommand(ShooterSubsystem shooter, TransportSubsystem transport) {
        m_shooter = shooter;
        m_transport = transport;
        addRequirements(shooter, transport);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        m_shooter.runForward();
        m_transport.runForward();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_shooter.stop();
        m_transport.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(kShootDurationSeconds);
    }
}
