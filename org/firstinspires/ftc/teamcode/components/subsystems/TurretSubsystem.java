package org.firstinspires.ftc.teamcode.components.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TurretSubsystem
 * Command-based turret control for 1 rotation (±180° range) using a GoBilda motor.
 */
public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;
    private final ElapsedTime timer = new ElapsedTime(); 

    // --- Motor and mechanical constants ---
    private static final double TICKS_PER_REV = 537.6;  // e.g., GoBilda 5202-0002-0027 motor
    private static final double GEAR_RATIO = 1.0;       // Adjust if turret is geared
    private final double ticksPerDegree = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // --- PID constants ---
    private double kP = 0.02;
    private double kI = 0.00002;
    private double kD = 0.0015;

    // --- PID state ---
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;

    // --- Control state ---
    private double targetAngle = 0;
    private boolean closedLoop = true;

    private final double maxPower = 0.1;

    // --- Motion limits ---
    // Allows one full rotation total: -180° to +180°
    private final double minAngle = -180.0;
    private final double maxAngle = 180.0;

    public TurretSubsystem(HardwareMap hardwareMap, String turretMotorName) {
        turretMotor = hardwareMap.get(DcMotorEx.class, turretMotorName);
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lastTime = timer.seconds();
    }

    // ---------------- Public API ----------------

    /** Set target turret angle (degrees), clamped to ±180° */
    public void setTargetAngle(double angleDeg) {
        targetAngle = clamp(angleDeg, minAngle, maxAngle);
        closedLoop = true;
    }

    /** Manual control with built-in safety limits */
    public void setManualPower(double power) {
        double current = getCurrentAngle();

        // Prevent exceeding limits while manually driving
        if ((current >= maxAngle && power > 0) || (current <= minAngle && power < 0)) {
            turretMotor.setPower(0);
        } else {
            turretMotor.setPower(clamp(power, -1, 1));
        }

        closedLoop = false;
    }

    /** Hold current position using PID */
    public void holdPosition() {
        closedLoop = true;
        targetAngle = getCurrentAngle();
        resetPID();
    }

    /** Reset encoder and treat current position as 0° (front) */
    public void zeroEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        targetAngle = 0;
        resetPID();
    }

    /** Get current turret angle (degrees) */
    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / ticksPerDegree;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    /** Periodic PID update (called by scheduler each loop) */
    @Override
    public void periodic() {
        updatePID();
    }

    private void updatePID() {
        if (!closedLoop) return;

        double now = timer.seconds();
        double dt = now - lastTime;
        if (dt <= 0) dt = 0.01;

        double error = targetAngle - getCurrentAngle();
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = kP * error + kI * integral + kD * derivative;
        output = clamp(output, -maxPower, maxPower);

        // Prevent exceeding limits under PID control
        double current = getCurrentAngle();
        if ((current >= maxAngle && output > 0) || (current <= minAngle && output < 0)) {
            output = 0;
        }

        turretMotor.setPower(output);
        lastError = error;
        lastTime = now;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        lastTime = timer.seconds();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void initializeTurret() {
    }
}
