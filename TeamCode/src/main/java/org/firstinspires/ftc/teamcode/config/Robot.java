package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystem.Intake;
import org.firstinspires.ftc.teamcode.config.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import java.util.List;

/**
 * FILE: Robot.java
 * PURPOSE: Central hub for all robot hardware.
 * WHY: Instead of passing 'hardwareMap' everywhere, we pass this 'Robot' object.
 */
public class Robot {
    // Public subsystems
    public final MecanumDrive drive;
    public final Shooter shooter;
    public final Intake intake;

    // Pedro Pathing Follower
    public final Follower f;

    // Hubs (Control/Expansion) used for Bulk Caching
    private final List<LynxModule> allHubs;

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        // Initialize Subsystems
        drive = new MecanumDrive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize Pedro Pathing
        f = Constants.createFollower(hardwareMap);

        // SETUP BULK READS
        // This sets the mode to MANUAL, meaning we must call clearBulkCache() ourselves.
        // This is much faster than the default AUTO mode.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    /**
     * Call this at the start of every loop()!
     * It clears old sensor data and updates PID controllers.
     */
    public void periodic() {
        // Clear cache so we get fresh data
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        // Update Pedro Pathing
        f.update();

        // Update Shooter PID logic
        shooter.periodic();
    }
}