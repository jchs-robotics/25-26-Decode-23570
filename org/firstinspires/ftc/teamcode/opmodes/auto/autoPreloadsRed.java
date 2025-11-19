package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.TurretSubsystem;

@Autonomous(name = "Auto Preloads Red")
public class autoPreloadsRed extends OpMode {

    // Subsystems
    private IndexSubsystem indexSubsystem;
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private DriveSubsystem driveSubsystem;
    private ShooterSubsystem shooterSubsystem;


    // Timer
    private final ElapsedTime timer = new ElapsedTime();


    // Initialization
    @Override
    public void init() {
        // Initialize all subsystems
        indexSubsystem = new IndexSubsystem(hardwareMap, "indexMotor");
        turretSubsystem = new TurretSubsystem(hardwareMap, "turretMotor");
        intakeSubsystem = new IntakeSubsystem(hardwareMap, "intakeMotor");
        driveSubsystem = new DriveSubsystem(hardwareMap, "frontleft", "backleft", "frontright", "backright", "imu");
        shooterSubsystem = new ShooterSubsystem(hardwareMap, "shooterMotor");

        indexSubsystem.initializeIndex();
        turretSubsystem.initializeTurret();
        intakeSubsystem.initializeIntake();
        driveSubsystem.initializeDrive();
        shooterSubsystem.initializeShooter();

        telemetry.addLine("Auto Preloads initialized — ready to start");
        telemetry.update();
    }


    // Start
    @Override
    public void start() {
        timer.reset();  // Start the timer when the match begins
    }


    // Main Loop
    @Override
    public void loop() {
        double elapsed = timer.seconds();

        // --- Shooter runs for first 30 seconds ---
        if (elapsed < 30) {
            shooterSubsystem.runShooter(0.95);
        } else {
            shooterSubsystem.stopShooter();
        }

        // Drive forward for 1 second
        if (elapsed < 1) {
            // Wait 20 seconds before moving
            driveSubsystem.Drive(0, 0, 0, 0);
        } else if (elapsed < 3) {
            // Drive forward for 1 seconds after waiting
            driveSubsystem.Drive(1.0, -1.0, -1.0, 1.0);
        } else {
            // Stop after driving
            driveSubsystem.Drive(0, 0, 0, 0);
        }

        // --- Index and Intake Sequence ---
        if (elapsed < 4) {
            // 0–5 sec: Shooter only
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 5.0) {
            // 6–6.1 sec: Run index
            indexSubsystem.runIndex(1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 5.1) {
            // 6.5–11.5 sec: Stop index
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 10.0) {
            // 11.5–14.5 sec: Run index again
            indexSubsystem.runIndex(1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 13.0) {
            // 14.5–19.5 sec: Stop index
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 14) {
            // 19.5–21 sec: Run intake
            intakeSubsystem.runIntake(-1.0);
            indexSubsystem.stopIndex();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 17.0) {
            // 21–25 sec: Run index again
            indexSubsystem.runIndex(1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 20.0) {
            // Stop intake
            intakeSubsystem.stopIntake();
            indexSubsystem.stopIndex();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 24.0) {
            // Drive forward for 0.5 seconds
            driveSubsystem.Drive(0.20, 0.2, 0.20, 0.20);

        } else if (elapsed < 24.5) {
            // stop
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 25.0) {
            // turn
            driveSubsystem.Drive(-0.20, -0.20, 0.20, 0.20);

        } else if (elapsed < 25.5) {
            // stop
            driveSubsystem.Drive(0, 0, 0, 0);

        } else {
            // stop everything
            driveSubsystem.Drive(0, 0, 0, 0);
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
        }


        // Telemetry
        telemetry.addData("Time", "%.1f sec", elapsed);
        telemetry.addData("Shooter", elapsed < 30 ? "ON" : "OFF");
        telemetry.addData("Index",
                (elapsed >= 5 && elapsed < 5.1)
                        || (elapsed >= 10.0 && elapsed < 13.0)
                        || (elapsed >= 21 && elapsed < 25)
                        ? "ON" : "OFF");
        telemetry.addData("Intake", (elapsed >= 14.0 && elapsed < 20) ? "ON" : "OFF");
        telemetry.addData("Drive", (elapsed >= 25 && elapsed < 25.5) ? "DRIVING" : "STOPPED");
        telemetry.update();
    }


    // Stop
    @Override
    public void stop() {
        // Stop everything just in case
        shooterSubsystem.stopShooter();
        intakeSubsystem.stopIntake();
        indexSubsystem.stopIndex();
        driveSubsystem.Drive(0, 0, 0, 0);
    }
}
