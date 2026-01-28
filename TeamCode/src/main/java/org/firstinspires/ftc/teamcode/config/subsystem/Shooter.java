package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class Shooter extends SubsystemBase {
    public final DcMotorEx launcher;
    private final Servo angle;
    private final Servo blocker;

    // --- TUNING ---
    public static double P = 800;
    public static double I = 0;
    public static double D = 0;
    public static double F = 25;

    public static double TARGET_VELOCITY = 1550;

    // --- POSITIONS ---
    // 0.85 = Closed (Default)
    // 0.24 = Open (Shooting)
    public double blockPos = 0.89;
    public double unblockPos = 0.24;

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angle = hardwareMap.get(Servo.class, "angle");

        // Initialize Blocker
        blocker = hardwareMap.get(Servo.class, "test_block_servo");
        blocker.setPosition(blockPos);

        // FLOAT = Coast
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        updatePIDF();
    }

    public void updatePIDF() {
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));
    }

    @Override
    public void periodic() {
        updatePIDF();
    }

    // --- ACTIONS ---
    public void setAngle(double pos) { angle.setPosition(pos); }
    public double getAngle() { return angle.getPosition(); }

    // Blocker
    public void block() { blocker.setPosition(blockPos); }
    public void unblock() { blocker.setPosition(unblockPos); }

    public void spinHigh() { launcher.setVelocity(TARGET_VELOCITY); }
    public void spinLow() { launcher.setVelocity(1200); }

    public void reverse() {
        launcher.setVelocity(-1125);
    }

    public void stop() {
        launcher.setPower(0); // Coast
    }

    public double getVelocity() { return launcher.getVelocity(); }
}