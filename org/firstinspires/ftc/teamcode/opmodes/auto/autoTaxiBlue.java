package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.TurretSubsystem;

@Autonomous(name = "Auto Taxi Blue")
public class autoTaxiBlue extends CommandOpMode {

    private IndexSubsystem indexSubsystem;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void initialize() {

        indexSubsystem = new IndexSubsystem(hardwareMap, "indexMotor");
        turretSubsystem = new TurretSubsystem(hardwareMap, "turretMotor");
        intakeSubsystem = new IntakeSubsystem(hardwareMap, "intakeMotor");
        driveSubsystem = new DriveSubsystem(hardwareMap, "frontleft", "backleft", "frontright", "backright", "imu");

        driveSubsystem.initializeDrive();

        waitForStart();
        timer.reset();
    }

    @Override
    public void run() {
        double elapsed = timer.seconds();

        if (elapsed < 5) {
            // Wait 20 seconds before moving
            driveSubsystem.Drive(0, 0, 0, 0);
        } else if (elapsed < 7) {
            // Drive forward for 2 seconds after waiting
            driveSubsystem.Drive(-0.3, 0.3, 0.3, -0.3);
        } else {
            // Stop after driving
            driveSubsystem.Drive(0, 0, 0, 0);
        }
    }
}
