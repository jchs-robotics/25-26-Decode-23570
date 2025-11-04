package org.firstinspires.ftc.teamcode.components.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndexSubsystem extends SubsystemBase {

    private final DcMotor indexMotor;
    private double indexPower = 0.0;

    public IndexSubsystem(final HardwareMap hMap, final String indexMotorName) {
        indexMotor = hMap.get(DcMotor.class, indexMotorName);
        indexMotor.setDirection(DcMotor.Direction.FORWARD);
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setPower(0);
    }

    /**
     * Set indexer motor power (-1.0 to 1.0)
     */
    public void setIndexPower(double power) {
        indexPower = power;
        indexMotor.setPower(power);
    }

    /**
     * Stop the indexer.
     */
    public void stopIndex() {
        setIndexPower(0.0);
    }

    /**
     * Get the current index motor power (for telemetry).
     */
    public double getIndexPower() {
        return indexPower;
    }

    @Override
    public void periodic() {
        // Optional telemetry or logic
    }

    public void runIndex(double power) {
        setIndexPower(-1.0);
    }

    public void initializeIndex() {
    }

}
