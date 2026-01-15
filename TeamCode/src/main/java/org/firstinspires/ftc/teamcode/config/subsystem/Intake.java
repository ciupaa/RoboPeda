package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
public class Intake extends SubsystemBase {
    private final DcMotorEx intakeMotor;

    public static double INTAKE_VELO = 1200;
    public static double OUTTAKE_VELO = -1000;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure your PIDF here
        intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(15, 3, 0, 12));
    }

    public void intake() { intakeMotor.setVelocity(INTAKE_VELO); }
    public void outtake() { intakeMotor.setVelocity(OUTTAKE_VELO); }
    public void stop() { intakeMotor.setVelocity(0); }
    public double getVelocity() { return intakeMotor.getVelocity(); }
}