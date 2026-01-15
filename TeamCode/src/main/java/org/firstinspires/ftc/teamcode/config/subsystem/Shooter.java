package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx launcher;
    private final Servo angle;

    // --- DASHBOARD TUNING ---
    public static double P = 800;
    public static double I = 0;
    public static double D = 0;
    public static double F = 13;

    // Velocities
    public static double TARGET_VELOCITY = 1550;
    public static double LOW_TARGET_VELOCITY = 1200;
    public static double REVERSE_VELOCITY = -1125;

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angle = hardwareMap.get(Servo.class, "angle");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void setAngle(double pos) { angle.setPosition(pos); }
    public double getAngle() { return angle.getPosition(); }

    public void spinHigh() { launcher.setVelocity(TARGET_VELOCITY); }
    public void spinLow() { launcher.setVelocity(LOW_TARGET_VELOCITY); }
    public void reverse() { launcher.setVelocity(REVERSE_VELOCITY); }

    public void stop() { launcher.setVelocity(0); }
    public double getVelocity() { return launcher.getVelocity(); }
}