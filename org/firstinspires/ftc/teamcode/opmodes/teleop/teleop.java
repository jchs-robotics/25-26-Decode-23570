package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.components.commands.IndexCommand;
import org.firstinspires.ftc.teamcode.components.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.components.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.components.commands.SshooterCommand;
import org.firstinspires.ftc.teamcode.components.commands.TurretAutoAimCommand;
import org.firstinspires.ftc.teamcode.components.commands.TurretManualCommand;
import org.firstinspires.ftc.teamcode.components.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.SshooterSubsystem;
import org.firstinspires.ftc.teamcode.components.subsystems.TurretSubsystem;




import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;


@TeleOp(name = "Tele Op")
public class teleop extends CommandOpMode {

    private GamepadEx driveController;
    private GamepadEx manipulatorController;

    private DriveSubsystem driveSubsystem;
    private IMU imu;
    private IntakeSubsystem intakeSubsystem;
    private IndexSubsystem indexSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private SshooterSubsystem sshooterSubsystem;
    private TurretSubsystem turretSubsystem;

    private IndexCommand indexCommand;
    private IndexCommand indexReverseCommand;

    private ShooterCommand shooterRPMCommand;
    private ShooterCommand shooterPowerCommand;
    private SshooterCommand sshooterPowerCommand;

    private IntakeCommand intakeForwardCommand;
    private IntakeCommand intakeReverseCommand;
    private Limelight3A limelight;
    private LimelightSubsystem limelightSubsystem;
    private TurretAutoAimCommand turretAutoAimCommand;



    private boolean indexRunningForward = false;
    private boolean indexRunningReverse = false;
    private boolean shooterRunning = false;
    private boolean sshooterRunning = false;
    private boolean intakeRunning = false;
    private boolean intakeForward = true; // true = forward, false = reverse
    private boolean useRPMControl = true;

    private double targetRPM = 1620;   // Default shooting speed
    private double shooterPower = 0.99; // Backup open-loop power
    private double sshooterPower = -0.99;
    private double intakePower = 1.0;  // Default intake power
    private boolean intakeReverse = false;
    private boolean autoAimEnabled = false;
    private boolean lastRightTriggerState = false;
    private String sshooterMotorName;


    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        driveController = new GamepadEx(gamepad1);
        manipulatorController = new GamepadEx(gamepad2);

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap, "frontleft", "backleft", "frontright", "backright", "imu");
        driveSubsystem.initializeDrive();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
        imu.initialize(parameters);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, "intakeMotor");
        indexSubsystem = new IndexSubsystem(hardwareMap, "indexMotor");
        shooterSubsystem = new ShooterSubsystem(hardwareMap, "shooterMotor");
        sshooterSubsystem = new SshooterSubsystem(hardwareMap, "sshooterMotor");
        turretSubsystem = new TurretSubsystem(hardwareMap, "turretServo");

        // Default turret manual control on gamepad1
        turretSubsystem.setDefaultCommand(new TurretManualCommand(
                turretSubsystem,
                () -> gamepad1.dpad_left,
                () -> gamepad1.dpad_right
        ));

        // Index commands
        indexCommand = new IndexCommand(indexSubsystem, 1.0);
        indexReverseCommand = new IndexCommand(indexSubsystem, -1.0);

        // Shooter commands
        shooterRPMCommand = new ShooterCommand(shooterSubsystem, targetRPM);
        shooterPowerCommand = new ShooterCommand(shooterSubsystem, shooterPower, false);


        // Intake commands
        intakeForwardCommand = new IntakeCommand(intakeSubsystem, -intakePower);
        intakeReverseCommand = new IntakeCommand(intakeSubsystem, intakePower);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

// choose pipeline
        limelight.pipelineSwitch(0);

// IMPORTANT: must call start() or getLatestResult() returns null
        limelight.start();

        limelightSubsystem = new LimelightSubsystem(hardwareMap);

        turretAutoAimCommand = new TurretAutoAimCommand(
                turretSubsystem,
                limelightSubsystem
        );


    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        // --- FIELD-CENTRIC DRIVE ---
        if (gamepad1.start) driveSubsystem.resetYaw(); // re-zero orientation

        double x = driveController.getLeftX();
        double y = driveController.getLeftY();
        double rx = driveController.getRightX();


        driveSubsystem.fieldCentricDrive(x, y, rx);

        boolean triggerPressed = gamepad1.right_trigger > 0.5;

// toggle on/off
        if (triggerPressed && !lastRightTriggerState) {
            autoAimEnabled = !autoAimEnabled;

            if (autoAimEnabled) {
                schedule(turretAutoAimCommand);
            } else {
                if (turretAutoAimCommand.isScheduled()) {
                    turretAutoAimCommand.cancel();
                }
                turretSubsystem.setPower(0);
            }
        }

        lastRightTriggerState = triggerPressed;


        // --- Index Controls ---
        if (gamepad2.a && !indexRunningForward) {
            CommandScheduler.getInstance().schedule(indexCommand);
            indexRunningForward = true;
            indexRunningReverse = false;
        } else if (gamepad2.y && !indexRunningReverse) {
            CommandScheduler.getInstance().schedule(indexReverseCommand);
            indexRunningReverse = true;
            indexRunningForward = false;
        } else if (!gamepad2.a && !gamepad2.y) {
            if (indexCommand.isScheduled()) indexCommand.cancel();
            if (indexReverseCommand.isScheduled()) indexReverseCommand.cancel();
            indexSubsystem.stopIndex();
            indexRunningForward = false;
            indexRunningReverse = false;
        }

        // --- Shooter Controls ---
        double rightTrigger = gamepad2.right_trigger;
        double leftTrigger = gamepad2.left_trigger;

        if (rightTrigger > 0.5 && !shooterRunning) {
            targetRPM = 1620;
            shooterRPMCommand = new ShooterCommand(shooterSubsystem, targetRPM);
            if (useRPMControl) {
                CommandScheduler.getInstance().schedule(shooterRPMCommand);
            } else {
                CommandScheduler.getInstance().schedule(shooterPowerCommand);
            }
            shooterRunning = true;
        } else if (leftTrigger > 0.5 && shooterRunning) {
            if (shooterRPMCommand.isScheduled()) shooterRPMCommand.cancel();
            if (shooterPowerCommand.isScheduled()) shooterPowerCommand.cancel();
            shooterSubsystem.stopShooter();
            shooterRunning = false;
        }

        if (rightTrigger > 0.5 && !sshooterRunning) {

            sshooterRunning = true;

        } else if (leftTrigger > 0.5 && sshooterRunning) {
            if (sshooterPowerCommand.isScheduled()) sshooterPowerCommand.cancel();
            sshooterSubsystem.stopSshooter();
            sshooterRunning = false;
        }

        if (gamepad2.x) {
            useRPMControl = !useRPMControl;
            sleep(300);
        }

        if (gamepad2.dpad_up) {
            targetRPM += 200;
            shooterRPMCommand = new ShooterCommand(shooterSubsystem, targetRPM);
            sleep(200);
        } else if (gamepad2.dpad_down) {
            targetRPM = Math.max(1620, targetRPM - 250);
            shooterRPMCommand = new ShooterCommand(shooterSubsystem, targetRPM);
            sleep(200);
        } else if (gamepad2.b) {
            targetRPM = 1620;
            shooterRPMCommand = new ShooterCommand(shooterSubsystem, targetRPM);
            sleep(300);
        }

        // --- Intake Controls ---
        boolean rightBumper = gamepad2.right_bumper;
        boolean leftBumper = gamepad2.left_bumper;

        if (rightBumper && !intakeRunning) {
            CommandScheduler.getInstance().schedule(intakeForwardCommand);
            intakeRunning = true;
            intakeForward = true;
        } else if (leftBumper && !intakeRunning) {
            CommandScheduler.getInstance().schedule(intakeReverseCommand);
            intakeRunning = true;
            intakeReverse = false;
        } else if (!rightBumper && !leftBumper && intakeRunning) {
            if (intakeForwardCommand.isScheduled()) intakeForwardCommand.cancel();
            if (intakeReverseCommand.isScheduled()) intakeReverseCommand.cancel();
            intakeSubsystem.setIntake(0);
            intakeRunning = false;
        }

        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            double tx = result.getTx();     // degrees left/right
            double ty = result.getTy();     // degrees up/down
            double latency = result.getCaptureLatency() + result.getTargetingLatency();

            // Auto-steer turret when target seen
            double turretError = tx;   // if tx = positive → target is to the right

            // simple turret aim assist:
            if (Math.abs(tx) > 1.0) {
                turretSubsystem.setPower((int) (turretError * 0.02));
            } else {
                turretSubsystem.setPower(0);
            }

            telemetry.addData("LL tx", tx);
            telemetry.addData("LL ty", ty);
            telemetry.addData("LL Latency", latency);

        } else {
            telemetry.addData("Limelight", "No valid target");
        }


        // --- Turret Presets ---
//        if (gamepad1.y) {
//            schedule(new TurretSetAngleCommand(turretSubsystem, 0.0));
//        } else if (gamepad1.x) {
//            schedule(new TurretSetAngleCommand(turretSubsystem, -90.0));
//        } else if (gamepad1.b) {
//            schedule(new TurretSetAngleCommand(turretSubsystem, 90.0));
//        } else if (gamepad1.a) {
//            schedule(new TurretSetAngleCommand(turretSubsystem, 180.0));
//        }
//
//        if (gamepad1.back) {
//            turretSubsystem.zeroEncoder();
//        }

        // --- Telemetry ---
        telemetry.addData("Turret Angle", "%.1f", turretSubsystem.getCurrentAngle());

        telemetry.addData("Shooter Running", shooterRunning);
        telemetry.addData("Sshooter Running", sshooterRunning);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Using RPM Control", useRPMControl);
        telemetry.addData("Current RPM", shooterSubsystem.getCurrentRPM());
        telemetry.addData("Index Forward", indexRunningForward);
        telemetry.addData("Index Reverse", indexRunningReverse);
        telemetry.addData("Intake Running", intakeRunning);
        telemetry.addData("Intake Direction", intakeForward ? "Forward" : "Reverse");
        telemetry.addData("Drive Mode", "FIELD-CENTRIC"); // always field-centric
        telemetry.addData("LL Pipeline", limelight.getStatus().getPipelineIndex());
        telemetry.addData("Auto Aim", autoAimEnabled);
        telemetry.update();
    }
}
