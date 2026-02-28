package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.Robot_v2;
import org.firstinspires.ftc.teamcode.config.util.Alliance;
import org.firstinspires.ftc.teamcode.config.util.Motif;
import org.firstinspires.ftc.teamcode.config.util.MotifScanHelper;

/**
 * Template autonomous for rebuilt robot (Robot_v2).
 *
 * 1. At start: turret rotates to AprilTag scan position, HuskyLens reads tag, motif is set on spindexer.
 * 2. Spindexer feed order will match motif (GPP, PGP, or PPG) for the rest of the match.
 *
 * Add your paths and shooting/intake logic after the motif scan. Use r.spindexer.getMotif(),
 * r.spindexer.rotateToNextFeedSlot(), and r.spindexer.advanceFeedPosition() when feeding in motif order.
 */
@Autonomous(name = "Auto Turret+Spindexer (Motif Scan)", group = "Competition")
public class Auto_TurretSpindexer_MotifScanTemplate extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot_v2 r = new Robot_v2(hardwareMap, Alliance.RED); // or Alliance.BLUE
        r.turretShooter.block();

        telemetry.addLine("Wait for start...");
        telemetry.update();
        waitForStart();

        // --- 1. Motif scan: turret to scan angle, read AprilTag, set motif ---
        telemetry.addLine("Scanning AprilTag for motif...");
        telemetry.update();
        Motif motif = MotifScanHelper.runMotifScan(r);
        telemetry.addLine("Motif: " + motif.name());
        telemetry.update();

        // --- 2. Your auto logic here ---
        // e.g. r.f.setStartingPose(...);
        // schedule( new SequentialCommandGroup( new FollowPath(...), ... ) );
        // When feeding to shooter in motif order:
        //   r.spindexer.rotateToNextFeedSlot();
        //   ... feed one artifact ...
        //   r.spindexer.advanceFeedPosition();

        while (opModeIsActive()) {
            r.periodic();
            telemetry.addData("Motif", motif.name());
            telemetry.update();
            sleep(20);
        }
    }
}
