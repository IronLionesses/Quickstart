package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lr");
        c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.662047018216351);
        c.yPodOffset.set(-0.4212044167706347);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27204566693885024);
                Controller secondaryTranslationalForward = Controller.proportional(0.10051360473980589);
                Controller primaryTranslationalLateral = Controller.proportional(0.4125967705348987);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1524434819238899);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011705566159108442));
                c.brake.set(Controller.proportionalFeedforward(0.009949731235242176));

                c.headingFeedback.set(Controller.proportional(4.859156989676669));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06170534929951028, 0.006378058021460639));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11238171761409352, 0.0554034109328655));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013425896100013558, 0.001848766090835212));

                c.maxAchievableForwardVelocity.set(74.47699159765878);
                c.maxAchievableStrafeVelocity.set(62.756669670173196);
                c.naturalForwardDeceleration.set(33.034528410005265);
                c.naturalStrafeDeceleration.set(60.51068050471802);
            }
    );

    // Builds the follower from the tuned configs above.
    // If you get a type error here, swap the first two arguments
    // (the official docs use localizer, drivetrain, algorithm).
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}