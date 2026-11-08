package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "shooterTest")
public class shooterTest extends LinearOpMode {

    private DcMotorEx shooter;

    // Target shooter speed (RPM)
    private static final double SHOOTER_RPM = 4000;

    // Replace with motor's encoder ticks per revolution
    private static final double TICKS_PER_REV = 28;

    @Override
    public void runOpMode() {

        // Connect shooter motor
        shooter = hardwareMap.get(
                DcMotorEx.class, "shooterTest"
        );

        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        // Convert RPM into encoder ticks per second
        double targetVelocity =
                (SHOOTER_RPM * TICKS_PER_REV) / 60.0;

        telemetry.addLine("BioBUZZ Shooter Ready");
        telemetry.addData("Target RPM", SHOOTER_RPM);
        telemetry.update();

        waitForStart();

        // CONTINUOUS SHOOTER - NO IDLE STATE
        if (opModeIsActive()) {

            shooter.setVelocity(targetVelocity);

            while (opModeIsActive()) {

                telemetry.addData(
                        "Shooter Velocity",
                        shooter.getVelocity()
                );

                telemetry.addData(
                        "Target Velocity",
                        targetVelocity
                );

                telemetry.update();
            }
        }

        // Safe shutdown when OpMode ends
        shooter.setVelocity(0);
    }
}

