package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final Timer timer = new Timer();
    private final double targetAngle;
    private final boolean isHighShot;
    private boolean hasFed = false;

    // Constructor: Passes the subsystem and settings
    public ShootCommand(Shooter shooter, boolean isHighShot, double targetAngle) {
        this.shooter = shooter;
        this.isHighShot = isHighShot;
        this.targetAngle = targetAngle;

        // IMPORTANT: 'addRequirements' tells the scheduler we are using the Shooter
        addRequirements(shooter);
    }

    // Called once when the command starts
    @Override
    public void initialize() {
        // Set Angle
        shooter.setAngle(targetAngle);

        // Start Flywheel (Spin Up)
        if (isHighShot) shooter.spinHigh();
        else shooter.spinLow();

        // Ensure Blocked (Gate Down) while spinning up
        shooter.block();

        timer.resetTimer();
        hasFed = false;
    }

    // Called repeatedly (50 times/sec)
    @Override
    public void execute() {
        // Wait logic:
        // IF we haven't fed the ring yet...
        // AND (The motor is at correct speed OR we have waited too long/timeout)
        if (!hasFed && (Math.abs(shooter.getVelocity() - Shooter.TARGET_VELOCITY) < 50 || timer.getElapsedTimeSeconds() > 1.5)) {

            // NEW: Open the gate to shoot
            shooter.unblock();

            timer.resetTimer(); // Reset timer to count how long we keep gate open
            hasFed = true;
        }
    }

    // Returns true when the command is done
    @Override
    public boolean isFinished() {
        // We are done if we have triggered the gate AND 0.5 seconds have passed
        return hasFed && timer.getElapsedTimeSeconds() > 0.5;
    }

    // Called once when finished
    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // Turn off flywheel (Coasts)
        shooter.block(); // Close the gate
        shooter.setAngle(1.0); // Reset angle to Idle
    }
}