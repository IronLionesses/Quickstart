package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Intake {

    public static double INTAKE_POWER = 1.0;
    public static double FEED_POWER = 0.7;
    public static double RAPID_FIRE_POWER = 1.0;
    public static double REVERSE_POWER = -1.0;

    public enum Mode {
        OFF,
        INTAKE,
        REVERSE,
        FEED,
        RAPID_FIRE
    }

    private final DcMotor intakeMotor;
    private final DcMotor intakeMotor1;

    private Mode mode = Mode.OFF;
    private double commandedPower = 0.0;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor1 = hardwareMap.get(DcMotor.class, "intakeMotor1");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Uncomment/adjust if the motors are mounted opposite each other
        // intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        offMode();
        update();
    }

    public void offMode()       { mode = Mode.OFF; }
    public void intakeMode()    { mode = Mode.INTAKE; }
    public void reverseMode()   { mode = Mode.REVERSE; }
    public void feedMode()      { mode = Mode.FEED; }
    public void rapidFireMode() { mode = Mode.RAPID_FIRE; }

    public void update() {
        switch (mode) {
            case INTAKE:     commandedPower = INTAKE_POWER; break;
            case REVERSE:    commandedPower = REVERSE_POWER; break;
            case FEED:       commandedPower = FEED_POWER; break;
            case RAPID_FIRE: commandedPower = RAPID_FIRE_POWER; break;
            default:         commandedPower = 0.0; break;
        }

        intakeMotor.setPower(commandedPower);
        intakeMotor1.setPower(commandedPower);
    }

    public Mode getMode() { return mode; }
    public String getModeName() { return mode.name(); }
    public double getCommandedPower() { return commandedPower; }
}