package org.firstinspires.ftc.teamcode.teleOP;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Blocker;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@Configurable
@TeleOp(
        name = "1. Competition TeleOp",
        group = "TeleOp"
)
public class CompetitionTeleOp extends LinearOpMode {

    // =========================================================
    // PRECISION FEED SETTINGS
    // =========================================================

    public static long FEED_PULSE_TIME_MS = 100;
    public static long FEED_RECOVERY_TIME_MS = 250;

    // =========================================================
    // SUBSYSTEMS
    // =========================================================

    private Follower follower;

    private Shooter shooter;
    private Intake intake;
    private Blocker blocker;

    private List<LynxModule> hubs;

    // =========================================================
    // PRECISION FEED STATE
    // =========================================================

    private enum FeedState {
        IDLE,
        FEEDING,
        RECOVERING
    }

    private FeedState feedState = FeedState.IDLE;
    private long feedStateStartNanos = 0L;

    // =========================================================
    // RUN OPMODE
    // =========================================================

    @Override
    public void runOpMode() {

        try {

            initialiseRobot();
            showReadyTelemetry();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            shooter.shooterIdle();
            resetFeedSequence();

            while (opModeIsActive()) {

                clearBulkCache();

                boolean precisionFire =
                        gamepad1.right_trigger > 0.5;

                boolean rapidFire =
                        gamepad1.left_trigger > 0.5;

                boolean intakeRequested =
                        gamepad1.right_bumper;

                boolean reverseRequested =
                        gamepad1.left_bumper;

                boolean shooterOnRequested =
                        gamepad1.cross;

                boolean shooterIdleRequested =
                        gamepad1.circle;

                updateDrive();

                updateShooterMode(
                        precisionFire,
                        rapidFire,
                        intakeRequested,
                        reverseRequested,
                        shooterOnRequested,
                        shooterIdleRequested
                );

                shooter.update();

                updateIntakeAndBlocker(
                        precisionFire,
                        rapidFire,
                        intakeRequested,
                        reverseRequested
                );

                intake.update();

                updateTelemetry(
                        precisionFire,
                        rapidFire
                );
            }

        } finally {

            safeShutdown();
        }
    }

    // =========================================================
    // INITIALISATION
    // =========================================================

    private void initialiseRobot() {

        follower = Constants.create(hardwareMap);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        blocker = new Blocker(hardwareMap);

        hubs = hardwareMap.getAll(
                LynxModule.class
        );

        for (LynxModule hub : hubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        shooter.shooterOff();

        intake.offMode();
        intake.update();

        blocker.closeBlocker();

        resetFeedSequence();
    }

    // =========================================================
    // READY TELEMETRY
    // =========================================================

    private void showReadyTelemetry() {

        telemetry.addLine("ROBOT READY");
        telemetry.addLine();

        telemetry.addLine("Cross / A: Shooter ON");
        telemetry.addLine("Circle / B: Shooter IDLE");
        telemetry.addLine("Right Trigger: Precision Fire");
        telemetry.addLine("Left Trigger: Rapid Fire");
        telemetry.addLine("Right Bumper: Intake");
        telemetry.addLine("Left Bumper: Reverse Intake");

        telemetry.addLine();

        telemetry.addData(
                "Shoot / Idle",
                "%.0f / %.0f RPM",
                Shooter.SHOOT_RPM,
                Shooter.IDLE_RPM
        );

        telemetry.addData(
                "KF / KP",
                "%.7f / %.7f",
                Shooter.KF,
                Shooter.KP
        );

        telemetry.addData(
                "Motor to wheel ratio",
                "%.4f",
                Shooter.MOTOR_TO_WHEEL_RATIO
        );

        telemetry.addData(
                "Precision pulse",
                "%d ms",
                FEED_PULSE_TIME_MS
        );

        telemetry.addData(
                "Precision recovery",
                "%d ms",
                FEED_RECOVERY_TIME_MS
        );

        telemetry.update();
    }

    // =========================================================
    // BULK CACHE
    // =========================================================

    private void clearBulkCache() {

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    // =========================================================
    // DRIVE
    // =========================================================

    private void updateDrive() {

        double forward =
                -gamepad1.left_stick_y;

        double strafe =
                -Math.pow(
                        gamepad1.left_stick_x,
                        3
                );

        double turn =
                -Math.pow(
                        gamepad1.right_stick_x,
                        3
                );

        follower.manual(
                new DrivePowers(
                        forward,
                        strafe,
                        turn
                )
        );

        follower.update();
    }

    // =========================================================
    // SHOOTER CONTROL
    // =========================================================

    private void updateShooterMode(
            boolean precisionFire,
            boolean rapidFire,
            boolean intakeRequested,
            boolean reverseRequested,
            boolean shooterOnRequested,
            boolean shooterIdleRequested
    ) {

        if (precisionFire
                || rapidFire
                || shooterOnRequested) {

            shooter.shooterOn();

        } else if (shooterIdleRequested
                || intakeRequested
                || reverseRequested) {

            shooter.shooterIdle();
        }
    }

    // =========================================================
    // INTAKE AND BLOCKER CONTROL
    // =========================================================

    private void updateIntakeAndBlocker(
            boolean precisionFire,
            boolean rapidFire,
            boolean intakeRequested,
            boolean reverseRequested
    ) {

        if (rapidFire) {

            resetFeedSequence();

            blocker.openBlocker();
            intake.rapidFireMode();

            return;
        }

        if (precisionFire) {

            updatePrecisionFeed();

            if (feedState == FeedState.FEEDING) {

                blocker.openBlocker();
                intake.feedMode();

            } else {

                blocker.closeBlocker();
                intake.offMode();
            }

            return;
        }

        resetFeedSequence();
        blocker.closeBlocker();

        if (intakeRequested) {

            intake.intakeMode();

        } else if (reverseRequested) {

            intake.reverseMode();

        } else {

            intake.offMode();
        }
    }

    // =========================================================
    // PRECISION FEED CONTROL
    // =========================================================

    private void updatePrecisionFeed() {

        switch (feedState) {

            case IDLE:

                setFeedState(
                        FeedState.FEEDING
                );

                break;

            case FEEDING:

                if (getFeedStateElapsedMs()
                        >= getSafeFeedPulseTimeMs()) {

                    setFeedState(
                            FeedState.RECOVERING
                    );
                }

                break;

            case RECOVERING:

                if (getFeedStateElapsedMs()
                        >= getSafeRecoveryTimeMs()) {

                    setFeedState(
                            FeedState.FEEDING
                    );
                }

                break;

            default:

                resetFeedSequence();
                break;
        }
    }

    private void setFeedState(
            FeedState newState
    ) {

        if (feedState == newState) {
            return;
        }

        feedState = newState;

        feedStateStartNanos =
                System.nanoTime();
    }

    private void resetFeedSequence() {

        feedState = FeedState.IDLE;
        feedStateStartNanos = 0L;
    }

    private double getFeedStateElapsedMs() {

        if (feedStateStartNanos == 0L) {
            return 0.0;
        }

        return (
                System.nanoTime()
                        - feedStateStartNanos
        ) / 1_000_000.0;
    }

    private long getSafeFeedPulseTimeMs() {

        return Math.max(
                1L,
                FEED_PULSE_TIME_MS
        );
    }

    private long getSafeRecoveryTimeMs() {

        return Math.max(
                0L,
                FEED_RECOVERY_TIME_MS
        );
    }

    // =========================================================
    // TELEMETRY
    // =========================================================

    private void updateTelemetry(
            boolean precisionFire,
            boolean rapidFire
    ) {

        String firingMode = "NONE";

        if (rapidFire) {

            firingMode = "RAPID FIRE";

        } else if (precisionFire) {

            firingMode = "PRECISION";
        }

        telemetry.addData(
                "Shooter Mode",
                shooter.getModeName()
        );

        telemetry.addData(
                "Firing Mode",
                firingMode
        );

        telemetry.addData(
                "Feed State",
                feedState.name()
        );

        telemetry.addData(
                "Feed Timer",
                "%.0f ms",
                getFeedStateElapsedMs()
        );

        telemetry.addData(
                "Intake",
                "%s | %.2f",
                intake.getModeName(),
                intake.getCommandedPower()
        );

        telemetry.addLine();

        telemetry.addData(
                "Shooter",
                "%.0f / %.0f RPM",
                shooter.getRPM(),
                shooter.getTargetRPM()
        );

        telemetry.addData(
                "Shooter Error",
                "%.0f RPM",
                shooter.getErrorRPM()
        );

        telemetry.addData(
                "Shooter Power",
                "%.3f",
                shooter.getCommandedPower()
        );

        telemetry.addLine();

        telemetry.addData(
                "Battery",
                "%.2f V",
                shooter.getBatteryVoltage()
        );

        telemetry.addData(
                "Voltage Compensation",
                "%.3f",
                shooter.getVoltageCompensation()
        );

        telemetry.addLine();

        telemetry.addData(
                "Shoot / Idle",
                "%.0f / %.0f RPM",
                Shooter.SHOOT_RPM,
                Shooter.IDLE_RPM
        );

        telemetry.addData(
                "KF / KP",
                "%.7f / %.7f",
                Shooter.KF,
                Shooter.KP
        );

        telemetry.addData(
                "Wheel Ratio",
                "%.4f",
                Shooter.MOTOR_TO_WHEEL_RATIO
        );

        telemetry.addData(
                "Feed Pulse / Recovery",
                "%d / %d ms",
                FEED_PULSE_TIME_MS,
                FEED_RECOVERY_TIME_MS
        );

        telemetry.addData(
                "Intake / Feed / Rapid",
                "%.2f / %.2f / %.2f",
                Intake.INTAKE_POWER,
                Intake.FEED_POWER,
                Intake.RAPID_FIRE_POWER
        );

        addPowerLimitWarning();

        telemetry.update();
    }

    // =========================================================
    // POWER LIMIT WARNING
    // =========================================================

    private void addPowerLimitWarning() {

        double safePowerLimit =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                Shooter.MAX_POWER
                        )
                );

        if (shooter.isShooting()
                && safePowerLimit > 0.0
                && shooter.getCommandedPower()
                >= safePowerLimit - 0.001) {

            telemetry.addLine(
                    "WARNING: SHOOTER AT POWER LIMIT"
            );
        }
    }

    // =========================================================
    // SAFE SHUTDOWN
    // =========================================================

    private void safeShutdown() {

        resetFeedSequence();

        if (intake != null) {

            intake.offMode();
            intake.update();
        }

        if (blocker != null) {
            blocker.closeBlocker();
        }

        if (shooter != null) {
            shooter.shooterOff();
        }

        if (follower != null) {

            follower.manual(
                    new DrivePowers(
                            0.0,
                            0.0,
                            0.0
                    )
            );

            follower.update();
        }
    }
}