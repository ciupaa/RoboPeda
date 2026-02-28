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

/**
 * New shooter subsystem:
 * - Two flywheel motors
 * - One turret motor (rotates the shooter independently of the chassis)
 * - Angle and blocker servos similar to the existing Shooter
 *
 * This does NOT change the existing Shooter class; it is a parallel implementation
 * for your rebuilt robot.
 */
@Config
@Configurable
public class TurretedShooter extends SubsystemBase {

    // --- HARDWARE ---
    public final DcMotorEx leftFlywheel;
    public final DcMotorEx rightFlywheel;
    public final DcMotorEx turret;

    private final Servo angle;
    private final Servo blocker;

    // --- FLYWHEEL PIDF (shared for both motors) ---
    public static double FLYWHEEL_P = 800;
    public static double FLYWHEEL_I = 0;
    public static double FLYWHEEL_D = 0;
    public static double FLYWHEEL_F = 25;

    public static double HIGH_TARGET_VELOCITY = 1550;
    public static double LOW_TARGET_VELOCITY = 1200;

    // --- BLOCKER SERVO POSITIONS ---
    public static double BLOCK_POS = 0.89;   // Closed
    public static double UNBLOCK_POS = 0.24; // Open (feeding)

    // --- ANGLE CONTROL ---
    public static boolean USE_ANGLE_OVERRIDE = false;
    public static double MANUAL_ANGLE_OVERRIDE = 0.70;

    // --- TURRET GEOMETRY ---
    /**
     * Encoder ticks per degree of turret rotation.
     * Measure one full rotation in ticks, divide by 360.
     */
    public static double TURRET_TICKS_PER_DEGREE = 10.0;

    /**
     * Mechanical travel limits for the turret, relative to its zero reference.
     * Example: -90 to +90 degrees.
     */
    public static double TURRET_MIN_DEG = -120.0;
    public static double TURRET_MAX_DEG = 120.0;

    /**
     * Offset so that turretAngleDeg = 0 means "straight ahead" relative to chassis.
     * Tune this so that 0 deg points directly at the goal when the chassis is aligned.
     */
    public static double TURRET_ZERO_OFFSET_DEG = 0.0;

    public static double TURRET_MAX_POWER = 0.6;

    public TurretedShooter(HardwareMap hardwareMap) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "shooter_left");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "shooter_right");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        angle = hardwareMap.get(Servo.class, "angle");
        blocker = hardwareMap.get(Servo.class, "test_block_servo");

        // Flywheel setup
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        updateFlywheelPIDF();

        // Turret setup
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        // Default to safe positions
        blocker.setPosition(BLOCK_POS);
    }

    // --- FLYWHEEL CONTROL ---

    public void updateFlywheelPIDF() {
        PIDFCoefficients coeffs = new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    @Override
    public void periodic() {
        updateFlywheelPIDF();
    }

    public void spinHigh() {
        leftFlywheel.setVelocity(HIGH_TARGET_VELOCITY);
        rightFlywheel.setVelocity(HIGH_TARGET_VELOCITY);
    }

    public void spinLow() {
        leftFlywheel.setVelocity(LOW_TARGET_VELOCITY);
        rightFlywheel.setVelocity(LOW_TARGET_VELOCITY);
    }

    public void stopFlywheels() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    public double getAverageVelocity() {
        return (leftFlywheel.getVelocity() + rightFlywheel.getVelocity()) / 2.0;
    }

    // --- ANGLE / BLOCKER ---

    public void setAngle(double pos) {
        if (USE_ANGLE_OVERRIDE) {
            angle.setPosition(MANUAL_ANGLE_OVERRIDE);
        } else {
            angle.setPosition(pos);
        }
    }

    public double getAngle() {
        return angle.getPosition();
    }

    public void block() {
        blocker.setPosition(BLOCK_POS);
    }

    public void unblock() {
        blocker.setPosition(UNBLOCK_POS);
    }

    // --- TURRET CONTROL ---

    /**
     * Set turret angle in degrees relative to its zero reference.
     * Positive = counterclockwise, negative = clockwise (conventional math).
     */
    public void setTurretAngleDegrees(double angleDeg) {
        double clamped = Math.max(TURRET_MIN_DEG, Math.min(TURRET_MAX_DEG, angleDeg));
        int targetTicks = (int) Math.round(clamped * TURRET_TICKS_PER_DEGREE);

        turret.setTargetPosition(targetTicks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(Math.abs(TURRET_MAX_POWER));
    }

    /**
     * @return turret angle in degrees based on encoder ticks and TURRET_TICKS_PER_DEGREE.
     */
    public double getTurretAngleDegrees() {
        return turret.getCurrentPosition() / TURRET_TICKS_PER_DEGREE;
    }

    /**
     * Aim the turret at a target global heading using the robot's global heading.
     *
     * @param robotHeadingDeg current robot heading (field reference, degrees)
     * @param targetHeadingDeg desired global heading pointing at the goal (degrees)
     */
    public void aimUsingRobotHeading(double robotHeadingDeg, double targetHeadingDeg) {
        aimWithVelocityCompensation(robotHeadingDeg, targetHeadingDeg, 0.0);
    }

    /**
     * Aim with velocity compensation: base aim + turret offset (degrees).
     * Use ShotResult.turretOffsetDeg when shooting while moving.
     */
    public void aimWithVelocityCompensation(double robotHeadingDeg, double targetHeadingDeg, double turretOffsetDeg) {
        double delta = normalizeAngleDeg(targetHeadingDeg - robotHeadingDeg);
        double turretCommandDeg = delta + TURRET_ZERO_OFFSET_DEG + turretOffsetDeg;
        setTurretAngleDegrees(turretCommandDeg);
    }

    /**
     * Set flywheel velocity from calculator output (same units as setVelocity).
     */
    public void setFlywheelVelocity(double velocity) {
        leftFlywheel.setVelocity(velocity);
        rightFlywheel.setVelocity(velocity);
    }

    private static double normalizeAngleDeg(double angle) {
        double a = angle % 360.0;
        if (a > 180.0) a -= 360.0;
        if (a < -180.0) a += 360.0;
        return a;
    }
}

