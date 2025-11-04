package org.firstinspires.ftc.teamcode.components.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {

    private DcMotor FLDrive;
    private DcMotor BLDrive;
    private DcMotor FRDrive;
    private DcMotor BRDrive;
    private IMU imu;

    public double FLEncoder;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public DriveSubsystem(final HardwareMap hMap, final String FLName, final String BLName,
                          final String FRName, final String BRName, final String imuName) {
        FLDrive = hMap.get(DcMotor.class, FLName);
        BLDrive = hMap.get(DcMotor.class, BLName);
        FRDrive = hMap.get(DcMotor.class, FRName);
        BRDrive = hMap.get(DcMotor.class, BRName);

        imu = hMap.get(IMU.class, imuName);
    }

    public void initializeDrive() {
        imu.initialize(parameters);

        FLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BLDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Drive(double FLPower, double BLPower, double FRPower, double BRPower) {
        FLDrive.setPower(FLPower);
        BLDrive.setPower(BLPower);
        FRDrive.setPower(FRPower);
        BRDrive.setPower(BRPower);
    }

    public void stop() {
        Drive(0, 0, 0, 0);
    }

    @Override
    public void periodic() {
        FLEncoder = FLDrive.getCurrentPosition();
    }

    public boolean tolerance(double point) {
        return FLEncoder < (point + 2.5) && FLEncoder > (point - 2.5);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    // FIELD-CENTRIC DRIVE CONTROL
    public void fieldCentricDrive(double inputX, double inputY, double inputRX) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the joystick input according to the bot heading
        double rotX = inputX * Math.cos(-botHeading) - inputY * Math.sin(-botHeading);
        double rotY = inputX * Math.sin(-botHeading) + inputY * Math.cos(-botHeading);

        rotX *= 1.1; // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(inputRX), 1);

        double FLPower = (rotY + rotX + inputRX) / denominator;
        double BLPower = (rotY - rotX + inputRX) / denominator;
        double FRPower = (rotY - rotX - inputRX) / denominator;
        double BRPower = (rotY + rotX - inputRX) / denominator;

        Drive(FLPower, BLPower, FRPower, BRPower);
    }
}
