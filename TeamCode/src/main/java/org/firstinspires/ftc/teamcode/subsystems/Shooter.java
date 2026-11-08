package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Shooter {
    public static double SHOOT_RPM = 2500.0;
    public static double AT_SPEED_TOLERANCE_RPM = 50.0;


    public static double KF = 0.00025;

    public static double KP = 0.005;

    public static double BOOST_THRESHOLD_RPM = 200.0;

    public static double RPM_FILTER_ALPHA = 0.4;


    public static double TICKS_PER_REV = 28.0;

    public static double MOTOR_TO_WHEEL_RATIO =
            1.0;

    // =========================================================
    // POWER LIMIT
    // =========================================================

    public static double MAX_POWER = 1.0;

    // =========================================================
    // VOLTAGE COMPENSATION
    // =========================================================

    public static double NOMINAL_VOLTAGE = 13.0;

    public static double MIN_COMPENSATION_VOLTAGE = 11.0;
    public static double VOLTAGE_READ_INTERVAL_S = 0.25;

    // =========================================================
    // MOTOR
    // =========================================================

    private final HardwareMap hardwareMap;

    private final DcMotorEx shooterMotor;

    // =========================================================
    // MODE
    // =========================================================

    public enum Mode {
        OFF,
        SHOOTING
    }

    private Mode mode = Mode.OFF;

    // =========================================================
    // CACHED VALUES
    // =========================================================

    private final ElapsedTime voltageTimer =
            new ElapsedTime();

    private double batteryVoltage =
            NOMINAL_VOLTAGE;

    private double voltageCompensation =
            1.0;

    private double currentRPM =
            0.0;

    private double targetRPM =
            0.0;

    private double errorRPM =
            0.0;

    private double commandedPower =
            0.0;

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        shooterMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        shooterMotor.setDirection(
                DcMotor.Direction.FORWARD
        );

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        refreshVoltage();

        shooterOff();
    }

    // =========================================================
    // UPDATE
    // =========================================================

    public void update() {

        if (voltageTimer.seconds()
                >= VOLTAGE_READ_INTERVAL_S) {

            refreshVoltage();
        }

        updateRPM();

        if (mode == Mode.OFF) {

            targetRPM = 0.0;
            errorRPM = 0.0;
            commandedPower = 0.0;

            shooterMotor.setPower(0.0);

            return;
        }

        targetRPM = SHOOT_RPM;

        errorRPM =
                targetRPM - currentRPM;

        commandedPower =
                calculatePower(
                        targetRPM,
                        currentRPM
                );

        shooterMotor.setPower(
                commandedPower
        );
    }

    private double calculatePower(
            double targetRPM,
            double measuredRPM
    ) {

        if (targetRPM <= 0.0) {
            return 0.0;
        }

        double error =
                targetRPM - measuredRPM;

        if (error > BOOST_THRESHOLD_RPM) {
            return clip(
                    MAX_POWER,
                    MAX_POWER
            );
        }

        double power =
                targetRPM * KF * voltageCompensation
                        + error * KP;

        return clip(
                power,
                MAX_POWER
        );
    }
    private void refreshVoltage() {

        batteryVoltage =
                readBatteryVoltage();

        voltageCompensation =
                NOMINAL_VOLTAGE
                        / Math.max(
                        MIN_COMPENSATION_VOLTAGE,
                        batteryVoltage
                );

        voltageTimer.reset();
    }

    private double readBatteryVoltage() {

        double highestVoltage = 0.0;

        for (VoltageSensor sensor
                : hardwareMap.voltageSensor) {

            double voltage =
                    sensor.getVoltage();

            if (voltage > 0.0
                    && Double.isFinite(voltage)) {

                highestVoltage =
                        Math.max(
                                highestVoltage,
                                voltage
                        );
            }
        }

        return highestVoltage > 0.0
                ? highestVoltage
                : NOMINAL_VOLTAGE;
    }

    private void updateRPM() {

        double raw = readRawRPM();

        double alpha =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                RPM_FILTER_ALPHA
                        )
                );

        currentRPM =
                alpha * raw
                        + (1.0 - alpha) * currentRPM;
    }

    private double readRawRPM() {

        if (TICKS_PER_REV <= 0.0
                || MOTOR_TO_WHEEL_RATIO <= 0.0) {

            return 0.0;
        }

        double ticksPerSecond =
                Math.abs(
                        shooterMotor.getVelocity()
                );

        double motorRPM =
                ticksPerSecond
                        * 60.0
                        / TICKS_PER_REV;

        return motorRPM
                * MOTOR_TO_WHEEL_RATIO;
    }


    public void shooterOn() {
        mode = Mode.SHOOTING;
    }

    public void shooterOff() {

        mode = Mode.OFF;

        targetRPM = 0.0;
        errorRPM = 0.0;
        commandedPower = 0.0;

        shooterMotor.setPower(0.0);
    }


    public Mode getMode() {
        return mode;
    }

    public String getModeName() {
        return mode.name();
    }

    public boolean isShooting() {
        return mode == Mode.SHOOTING;
    }

    public boolean isOff() {
        return mode == Mode.OFF;
    }

    public boolean isAtSpeed() {
        return mode == Mode.SHOOTING
                && Math.abs(errorRPM)
                <= AT_SPEED_TOLERANCE_RPM;
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getErrorRPM() {
        return errorRPM;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getVoltageCompensation() {
        return voltageCompensation;
    }


    private double clip(
            double power,
            double maximumPower
    ) {

        double max =
                Math.max(
                        0.0,
                        Math.min(
                                1.0,
                                maximumPower
                        )
                );

        return Math.max(
                0.0,
                Math.min(
                        max,
                        power
                )
        );
    }
}