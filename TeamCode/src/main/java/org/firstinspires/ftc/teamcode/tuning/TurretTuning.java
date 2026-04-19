package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous(group = "tuning")
public class TurretTuning extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    private Turret turret;
    private IntakeAndTransfer intakeAndTransfer;

    private GamepadEx gamepad;

    private Stopwatch stopwatch;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();

        pinpoint = new Pinpoint(Transform.ZERO);
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        localizer = new FusedLocalizer(pinpoint, limelight, Transform.ZERO);

        turret = new Turret(voltageSensor);
        intakeAndTransfer = new IntakeAndTransfer();

        gamepad = new GamepadEx(gamepad1);

        stopwatch = new Stopwatch();

        register(voltageSensor, pinpoint, limelight, localizer, turret, intakeAndTransfer, gamepad);
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
        addData("X", localizer.getPosition().getX());
        addData("Y", localizer.getPosition().getY());
        addData("Theta", localizer.getPosition().getTheta());

        addData("X Cov", localizer.getCovariance().getEntry(0, 0));
        addData("Y Cov", localizer.getCovariance().getEntry(1, 1));
        addData("Theta Cov", localizer.getCovariance().getEntry(2, 2));

        addData("Target X", targetX);
        addData("Target Y", targetY);

        addData("RPM", rpm);
        addData("Actual RPM", turret.getVelocity());
        addData("Hood", hood);

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

        intakeAndTransfer.transfer(gamepad.getRightBumper());
        intakeAndTransfer.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger());
    }
}
