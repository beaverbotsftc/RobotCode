package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous(group = "tuning")
public class TurretLocationTuning extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    private Turret turret;

    private GamepadEx gamepad;

    private Stopwatch stopwatch;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();

        pinpoint = new Pinpoint(Transform.ZERO);
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        localizer = new FusedLocalizer(pinpoint, limelight, Transform.ZERO);

        turret = new Turret(voltageSensor);

        gamepad = new GamepadEx(gamepad1);

        stopwatch = new Stopwatch();

        register(voltageSensor, pinpoint, limelight, localizer, turret, gamepad);
    }

    @Override
    public void onStart() {
        stopwatch.reset();
    }

    double targetX = -70.3125;
    double targetY = 70.3125;
    double rpm = 0;
    double hood = 0;

    @Override
    public void periodic() {
        telemetry.addData("X", localizer.getPosition().getX());
        telemetry.addData("Y", localizer.getPosition().getY());
        telemetry.addData("Theta", localizer.getPosition().getTheta());

        telemetry.addData("X Cov", localizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Y Cov", localizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Theta Cov", localizer.getCovariance().getEntry(2, 2));

        telemetry.addData("Target X", targetX);
        telemetry.addData("Target Y", targetY);

        telemetry.addData("RPM", rpm);
        telemetry.addData("Hood", hood);

        if (gamepad.getDpadUp()) targetY += 0.0125;
        if (gamepad.getDpadDown()) targetY -= 0.0125;
        if (gamepad.getDpadLeft()) targetX -= 0.0125;
        if (gamepad.getDpadRight()) targetX += 0.0125;

        if (gamepad.getAJustPressed()) rpm -= 50;
        if (gamepad.getYJustPressed()) rpm += 50;
        if (gamepad.getXJustPressed()) hood -= 0.05;
        if (gamepad.getBJustPressed()) hood += 0.05;

        turret.turn(localizer.getPosition().relativeAngleTo(new Transform(targetX, targetY)));
        turret.shoot(rpm);
        turret.setHoodAngle(hood);
    }
}
