package org.firstinspires.ftc.teamcode.components.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor;

    // --- goBILDA 5204 Yellow Jacket 3.7:1 Motor ---
    private static final double TICKS_PER_REV = 103.8; // 28 * 3.7 (from goBILDA specs)
    private static final double GEAR_RATIO = 1.0;      // Adjust if extra gearing added
    private static final double RPM_TO_TICKS_PER_SEC = (TICKS_PER_REV * GEAR_RATIO) / 60.0; 

    private double targetRPM = 0.0;

    public ShooterSubsystem(final HardwareMap hMap, final String shooterName) {
        shooterMotor = hMap.get(DcMotorEx.class, shooterName);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setPower(0);
    }

    /** Set shooter RPM using built-in velocity PID */
    public void setShooterRPM(double rpm) {
        targetRPM = rpm;
        double ticksPerSecond = rpm * RPM_TO_TICKS_PER_SEC;
        shooterMotor.setVelocity(ticksPerSecond);
    }

    /** Open-loop control (direct power) */
    public void setShooterPower(double power) {
        targetRPM = 0;
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setPower(power);
    }

    /** Stop shooter motor */
    public void stopShooter() {
        shooterMotor.setPower(0);
        targetRPM = 0;
    }

    /** Get current RPM */
    public double getCurrentRPM() {
        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    @Override
    public void periodic() {

    }

    public void runShooter(double power) {
        shooterMotor.setPower(0.95);
    }

    private void runShooterPower(double v) {
    }

    public void initializeShooter() {
    }
}


