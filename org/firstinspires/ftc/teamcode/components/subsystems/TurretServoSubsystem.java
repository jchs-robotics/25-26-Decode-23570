package org.firstinspires.ftc.teamcode.components.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * TurretSubsystem (Continuous Servo Version)
 * NOTE: Continuous servos do NOT give position feedback.
 * Angle limits require external sensors (limit switch, magnetic switch, etc.)
 */
public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretServo;

    // Servo power limits (continuous servo uses -1 to +1)
    private final double maxPower = 0.1;

    // OPTIONAL: Only valid if you add limit switches or sensors
    private double minAngle = -180;
    private double maxAngle = 180;

    private boolean closedLoop = false;
    private double targetAngle = 0;
    private double currentAngle = 0;   // MUST be maintained by your own sensor!

    public TurretSubsystem(HardwareMap hardwareMap, String servoName) {
        turretServo = hardwareMap.get(CRServo.class, servoName);
        turretServo.setPower(0);
    }

    /** Manual power control (-1 to +1) */
    public void setManualPower(double power) {
        closedLoop = false;
        turretServo.setPower(clamp(power, -maxPower, maxPower));
    }

    /**
     * ⚠ Requires an external sensor to track angle.
     * If you don’t have one, this function CANNOT work.
     */
    public void setTargetAngle(double angle) {
        closedLoop = true;
        targetAngle = clamp(angle, minAngle, maxAngle);
    }

    /** Replace with your sensor-based angle */
    public double getCurrentAngle() {
        return currentAngle; // must update externally!
    }

    /** Call periodically to move toward angle */
    @Override
    public void periodic() {
        if (!closedLoop) return;

        double error = targetAngle - currentAngle;

        // Simple proportional drive, no encoder
        double power = 0.01 * error;

        power = clamp(power, -maxPower, maxPower);

        turretServo.setPower(power);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void initializeTurret() {}
}
