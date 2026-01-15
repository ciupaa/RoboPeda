package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Shooter extends SubsystemBase {
    private final DcMotorEx launcher;
    private final Servo angle;

    public static double TARGET_VELOCITY = 1450;
    public static double TOLERANCE = 50;

    public Shooter(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        angle = hardwareMap.get(Servo.class, "angle");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(600, 0, 0, 10));
    }

    public void setAngle(double position) { angle.setPosition(position); }
    public double getAngle() { return angle.getPosition(); }

    public void setTargetVelocity(double velocity) { TARGET_VELOCITY = velocity; }
    public void spinUp() { launcher.setVelocity(TARGET_VELOCITY); }
    public void stop() { launcher.setVelocity(0); }

    public boolean isReady() { return Math.abs(launcher.getVelocity() - TARGET_VELOCITY) < TOLERANCE; }
    public double getVelocity() { return launcher.getVelocity(); }
}