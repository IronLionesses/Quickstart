package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
@TeleOp
public class Shooter {

    // =========================================================
    // SHOOTING SETTINGS
    // =========================================================

    // Shooter wheel RPM, not motor RPM.
    public static double TARGET_RPM = 2800.0;

    // Approximate power required to maintain TARGET_RPM.
    public static double BASE_POWER = 0.70;

    // Shooting proportional gain.
    public static double KP = 0.0006;


    // =========================================================
    // IDLE SETTINGS
    // =========================================================

    // Shooter wheel RPM while idling.
    public static double IDLE_RPM = 1000.0;

    // Approximate power required to maintain IDLE_RPM.
    public static double IDLE_BASE_POWER = 0.25;

    // Idle proportional gain.
    public static double IDLE_KP = 0.00010;


    // =========================================================
    // ENCODER AND GEARING
    // =========================================================

    // Encoder ticks per motor revolution.
    public static double TICKS_PER_REV = 28.0;

    /*
     * shooterWheelRPM = motorRPM * GEAR_RATIO
     *
     * Direct drive:
     * GEAR_RATIO = 1.0
     *
     * 10-tooth motor sprocket driving a 14-tooth shooter sprocket:
     * GEAR_RATIO = 10.0 / 14.0
     */
    public static double GEAR_RATIO = 1.0;


    // =========================================================
    // READY SETTINGS
    // =========================================================

    public static double READY_TOLERANCE_RPM = 500.0;

    public static double READY_SETTLE_TIME_MS = 200.0;


    // =========================================================
    // MOTOR
    // =========================================================

    private final DcMotorEx shooterMotor;


    // =========================================================
    // SHOOTER MODE
    // =========================================================

    private enum ShooterMode {
        OFF,
        IDLE,
        SHOOTING
    }

    private ShooterMode mode = ShooterMode.OFF;


    // =========================================================
    // CONTROL STATE
    // =========================================================

    private double activeTargetRPM = 0.0;
    private double measuredRPM = 0.0;
    private double errorRPM = 0.0;
    private double commandedPower = 0.0;

    private long readySinceNanos = 0L;


    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    public Shooter(HardwareMap hardwareMap) {

        shooterMotor = hardwareMap.get(
                DcMotorEx.class,
                "shooterMotor"
        );

        shooterMotor.setDirection(
                DcMotor.Direction.REVERSE
        );

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooterMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterOff();
    }


    // =========================================================
    // UPDATE
    // =========================================================

    /**
     * Call once during every control loop after selecting the
     * desired shooter mode.
     */
    public void update() {

        // Read the shooter RPM once per loop.
        measuredRPM = readRPM();

        // -----------------------------------------------------
        // OFF
        // -----------------------------------------------------

        if (mode == ShooterMode.OFF) {

            activeTargetRPM = 0.0;
            errorRPM = 0.0;
            commandedPower = 0.0;

            shooterMotor.setPower(0.0);

            resetReadyTimer();
            return;
        }

        // -----------------------------------------------------
        // SELECT MODE PARAMETERS
        // -----------------------------------------------------

        double selectedBasePower;
        double selectedKP;

        if (mode == ShooterMode.SHOOTING) {

            activeTargetRPM = TARGET_RPM;
            selectedBasePower = BASE_POWER;
            selectedKP = KP;

        } else {

            activeTargetRPM = IDLE_RPM;
            selectedBasePower = IDLE_BASE_POWER;
            selectedKP = IDLE_KP;
        }

        // -----------------------------------------------------
        // CALCULATE ERROR
        // -----------------------------------------------------

        errorRPM =
                activeTargetRPM
                        - measuredRPM;

        // -----------------------------------------------------
        // CALCULATE POWER
        // -----------------------------------------------------

        commandedPower = calculatePower(
                activeTargetRPM,
                measuredRPM,
                selectedBasePower,
                selectedKP
        );

        // -----------------------------------------------------
        // APPLY POWER
        // -----------------------------------------------------

        shooterMotor.setPower(commandedPower);
    }


    // =========================================================
    // P CONTROLLER
    // =========================================================

    private double calculatePower(
            double targetRPM,
            double currentRPM,
            double basePower,
            double kP
    ) {

        if (targetRPM <= 0.0) {
            return 0.0;
        }

        double currentErrorRPM =
                targetRPM
                        - currentRPM;

        double proportionalCorrection =
                currentErrorRPM
                        * kP;

        return clipPower(
                basePower
                        + proportionalCorrection
        );
    }


    // =========================================================
    // SHOOTER ON
    // =========================================================

    public void shooterOn() {

        if (mode != ShooterMode.SHOOTING) {
            resetReadyTimer();
        }

        mode = ShooterMode.SHOOTING;
    }


    // =========================================================
    // SHOOTER IDLE
    // =========================================================

    public void shooterIdle() {

        if (mode != ShooterMode.IDLE) {
            resetReadyTimer();
        }

        mode = ShooterMode.IDLE;
    }


    // =========================================================
    // SHOOTER OFF
    // =========================================================

    public void shooterOff() {

        mode = ShooterMode.OFF;

        activeTargetRPM = 0.0;
        measuredRPM = 0.0;
        errorRPM = 0.0;
        commandedPower = 0.0;

        shooterMotor.setPower(0.0);

        resetReadyTimer();
    }


    // =========================================================
    // RPM READING
    // =========================================================

    private double readRPM() {

        double ticksPerSecond =
                Math.abs(
                        shooterMotor.getVelocity()
                );

        return ticksPerSecondToWheelRPM(
                ticksPerSecond,
                GEAR_RATIO
        );
    }


    // =========================================================
    // RPM CONVERSION
    // =========================================================

    private double ticksPerSecondToWheelRPM(
            double ticksPerSecond,
            double gearRatio
    ) {

        if (TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        if (gearRatio <= 0.0) {
            return 0.0;
        }

        double motorRPM =
                ticksPerSecond
                        * 60.0
                        / TICKS_PER_REV;

        return motorRPM
                * gearRatio;
    }


    // =========================================================
    // READY CHECK
    // =========================================================

    /**
     * Checks whether the supplied RPM is currently within the
     * shooting tolerance.
     */
    public boolean isReady(double rpm) {

        if (mode != ShooterMode.SHOOTING) {
            return false;
        }

        double absoluteErrorRPM =
                Math.abs(
                        TARGET_RPM
                                - rpm
                );

        return absoluteErrorRPM
                <= READY_TOLERANCE_RPM;
    }


    /**
     * Checks whether the most recently measured RPM is currently
     * within the shooting tolerance.
     */
    public boolean isReady() {
        return isReady(measuredRPM);
    }


    // =========================================================
    // SHOOTER READY
    // =========================================================

    /**
     * Returns true once the shooter has remained within the ready
     * tolerance continuously for READY_SETTLE_TIME_MS.
     */
    public boolean shooterReady() {

        if (mode != ShooterMode.SHOOTING) {

            resetReadyTimer();
            return false;
        }

        if (!isReady(measuredRPM)) {

            resetReadyTimer();
            return false;
        }

        if (readySinceNanos == 0L) {

            readySinceNanos =
                    System.nanoTime();

            return false;
        }

        double stableTimeMilliseconds =
                (System.nanoTime()
                        - readySinceNanos)
                        / 1_000_000.0;

        return stableTimeMilliseconds
                >= READY_SETTLE_TIME_MS;
    }


    // =========================================================
    // RESET READY TIMER
    // =========================================================

    private void resetReadyTimer() {
        readySinceNanos = 0L;
    }


    // =========================================================
    // MODE GETTERS
    // =========================================================

    public boolean isShooting() {
        return mode == ShooterMode.SHOOTING;
    }


    public boolean isIdle() {
        return mode == ShooterMode.IDLE;
    }


    public boolean isOff() {
        return mode == ShooterMode.OFF;
    }


    public String getModeName() {
        return mode.name();
    }


    // =========================================================
    // RPM GETTERS
    // =========================================================

    /**
     * Returns the shooter wheel RPM captured during the most
     * recent update().
     */
    public double getRPM() {
        return measuredRPM;
    }


    /**
     * Returns the active shooter wheel target RPM.
     */
    public double getTargetRPM() {
        return activeTargetRPM;
    }


    /**
     * Returns the current target minus measured RPM error.
     */
    public double getErrorRPM() {
        return errorRPM;
    }


    // =========================================================
    // POWER GETTER
    // =========================================================

    public double getCommandedPower() {
        return commandedPower;
    }


    // =========================================================
    // RAW ENCODER VELOCITY
    // =========================================================

    /**
     * Returns the current raw encoder velocity in ticks per second.
     *
     * This performs a new hardware read.
     */
    public double getTicksPerSecond() {

        return Math.abs(
                shooterMotor.getVelocity()
        );
    }


    // =========================================================
    // MOTOR RPM
    // =========================================================

    /**
     * Returns motor shaft RPM before applying GEAR_RATIO.
     *
     * This performs a new hardware read.
     */
    public double getMotorRPM() {

        if (TICKS_PER_REV <= 0.0) {
            return 0.0;
        }

        return Math.abs(
                shooterMotor.getVelocity()
        )
                * 60.0
                / TICKS_PER_REV;
    }


    // =========================================================
    // POWER CLIPPING
    // =========================================================

    private double clipPower(double power) {

        return Math.max(
                0.0,
                Math.min(
                        1.0,
                        power
                )
        );
    }
}