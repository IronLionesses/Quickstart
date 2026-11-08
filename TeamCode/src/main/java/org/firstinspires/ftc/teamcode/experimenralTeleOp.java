
package org.firstinspires.ftc.teamcode;

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
    // SUBSYSTEMS
    // =========================================================

    private Follower follower;

    private Shooter shooter;
    private Intake intake;
    private Blocker blocker;

    private List<LynxModule> hubs;

    // Track shooter state to avoid unnecessary mode commands
    private boolean shooterEnabled = false;

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

            // Default shooter state
            setShooterState(true);

            while (opModeIsActive()) {

                // Clear manual bulk cache once per loop
                clearBulkCache();

                // =================================================
                // GAMEPAD INPUTS
                // =================================================

                boolean fire =
                        gamepad1.right_trigger > 0.5;

                boolean intakeRequested =
                        gamepad1.right_bumper;

                boolean reverseRequested =
                        gamepad1.left_bumper;

                boolean shooterOnRequested =
                        gamepad1.cross;

                boolean shooterOffRequested =
                        gamepad1.circle;

                // =================================================
                // UPDATE SUBSYSTEMS
                // =================================================

                updateDrive();

                updateShooterMode(
                        fire,
                        shooterOnRequested,
                        shooterOffRequested
                );

                shooter.update();

                updateIntakeAndBlocker(
                        fire,
                        intakeRequested,
                        reverseRequested
                );

                intake.update();

                updateTelemetry(fire);
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

        // Explicitly stop drivetrain
        follower.manual(
                new DrivePowers(
                        0.0,
                        0.0,
                        0.0
                )
        );

        follower.update();

        // Initialise shooter
        shooter.shooterOff();
        shooterEnabled = false;

        // Initialise intake
        intake.offMode();
        intake.update();

        // Initialise blocker
        blocker.closeBlocker();
    }

    // =========================================================
    // READY TELEMETRY
    // =========================================================

    private void showReadyTelemetry() {

        telemetry.addLine("ROBOT READY");
        telemetry.addLine();

        telemetry.addLine("Cross / A: Shooter ON");
        telemetry.addLine("Circle / B: Shooter OFF");
        telemetry.addLine("Right Trigger: Fire");
        telemetry.addLine("Right Bumper: Intake");
        telemetry.addLine("Left Bumper: Reverse Intake");

        telemetry.addLine();

        telemetry.addData(
                "Shoot RPM",
                "%.0f",
                Shooter.SHOOT_RPM
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

        // Cubic joystick scaling for consistent sensitivity

        double forward =
                -Math.pow(gamepad1.left_stick_y, 3);

        double strafe =
                -Math.pow(gamepad1.left_stick_x, 3);

        double turn =
                -Math.pow(gamepad1.right_stick_x, 3);

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
    // SHOOTER STATE MANAGEMENT
    // =========================================================

    private void setShooterState(boolean enabled) {

        if (shooterEnabled == enabled) {
            return;
        }

        shooterEnabled = enabled;

        if (enabled) {
            shooter.shooterOn();
        } else {
            shooter.shooterOff();
        }
    }

    // =========================================================
    // SHOOTER CONTROL
    // =========================================================

    private void updateShooterMode(
            boolean fire,
            boolean shooterOnRequested,
            boolean shooterOffRequested
    ) {

        // Firing requires shooter enabled
        if (fire || shooterOnRequested) {

            setShooterState(true);

        } else if (shooterOffRequested) {

            setShooterState(false);
        }
    }

    // =========================================================
    // INTAKE AND BLOCKER CONTROL
    // =========================================================

    private void updateIntakeAndBlocker(
            boolean fire,
            boolean intakeRequested,
            boolean reverseRequested
    ) {

        // Firing has priority over intake
        if (fire) {

            blocker.openBlocker();
            intake.rapidFireMode();

            return;
        }

        // Close blocker whenever not firing
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
    // TELEMETRY
    // =========================================================

    private void updateTelemetry(boolean fire) {

        telemetry.addData(
                "Shooter Mode",
                shooter.getModeName()
        );

        telemetry.addData(
                "Shooter Enabled",
                shooterEnabled ? "YES" : "NO"
        );

        telemetry.addData(
                "Firing",
                fire ? "YES" : "NO"
        );

        telemetry.addData(
                "Intake",
                "%s | %.2f",
                intake.getModeName(),
                intake.getCommandedPower()
        );

        telemetry.addLine();

        telemetry.addData(
                "Shooter RPM",
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
                "Shoot RPM",
                "%.0f",
                Shooter.SHOOT_RPM
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
                "Intake / Rapid",
                "%.2f / %.2f",
                Intake.INTAKE_POWER,
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

        // Each subsystem gets an independent shutdown attempt

        try {

            if (intake != null) {
                intake.offMode();
                intake.update();
            }

        } catch (Exception e) {
            telemetry.addData(
                    "Intake shutdown error",
                    e.getMessage()
            );
        }

        try {

            if (blocker != null) {
                blocker.closeBlocker();
            }

        } catch (Exception e) {
            telemetry.addData(
                    "Blocker shutdown error",
                    e.getMessage()
            );
        }

        try {

            if (shooter != null) {
                shooter.shooterOff();
                shooterEnabled = false;
            }

        } catch (Exception e) {
            telemetry.addData(
                    "Shooter shutdown error",
                    e.getMessage()
            );
        }

        try {

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

        } catch (Exception e) {
            telemetry.addData(
                    "Drive shutdown error",
                    e.getMessage()
            );
        }
    }
}

