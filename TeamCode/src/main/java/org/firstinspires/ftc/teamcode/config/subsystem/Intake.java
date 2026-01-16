package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake extends SubsystemBase {
    private final DcMotorEx intakeMotor;

    // --- SAFETY TUNING ---
    // 6.0 Amps (Stops if exceeded)
    public static double JAM_THRESHOLD = 6.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intake() { intakeMotor.setVelocity(1200); }
    public void outtakeSlow() { intakeMotor.setVelocity(-450); }
    public void outtakeSuperSlow() { intakeMotor.setVelocity(-250); }
    public void stop() { intakeMotor.setVelocity(0); }

    public double getCurrentDraw() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}