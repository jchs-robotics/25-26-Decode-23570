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
            shooterSubsystem.runShooter(-0.95);
        } else {
            shooterSubsystem.stopShooter();
        }

        // --- Index and Intake Sequence ---
        if (elapsed < 5) {
            // 0–5 sec: Shooter only
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 6.5) {
            // 5–6.5 sec: Run index
            indexSubsystem.runIndex(-1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 11.5) {
            // 6.5–11.5 sec: Stop index
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 14.5) {
            // 11.5–14.5 sec: Run index again
            indexSubsystem.runIndex(-1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 19.5) {
            // 14.5–19.5 sec: Stop index
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 21) {
            // 19.5–21 sec: Run intake
            intakeSubsystem.runIntake(-1.0);
            indexSubsystem.stopIndex();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 25) {
            // 21–25 sec: Run index again
            indexSubsystem.runIndex(-1.0);
            intakeSubsystem.stopIntake();
            driveSubsystem.Drive(0, 0, 0, 0);

        } else if (elapsed < 27) {
            // 25–27 sec: Drive backward
            driveSubsystem.Drive(0.3, -0.3, -0.3, 0.3);
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();

        } else {
            // 27–30 sec: Stop everything
            driveSubsystem.Drive(0, 0, 0, 0);
            indexSubsystem.stopIndex();
            intakeSubsystem.stopIntake();
        }


        // Telemetry
        telemetry.addData("Time", "%.1f sec", elapsed);
        telemetry.addData("Shooter", elapsed < 30 ? "ON" : "OFF");
        telemetry.addData("Index",
                (elapsed >= 5 && elapsed < 6.5)
                        || (elapsed >= 11.5 && elapsed < 14.5)
                        || (elapsed >= 21 && elapsed < 25)
                        ? "ON" : "OFF");
        telemetry.addData("Intake", (elapsed >= 19.5 && elapsed < 21) ? "ON" : "OFF");
        telemetry.addData("Drive", (elapsed >= 25 && elapsed < 27) ? "DRIVING" : "STOPPED");
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
