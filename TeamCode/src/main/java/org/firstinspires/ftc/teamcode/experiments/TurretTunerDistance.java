package org.firstinspires.ftc.teamcode.experiments;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TurretTunerDistance extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Intake intake;
    private Stopper stopper;
    private TurretV2 turret;
    private Pinpoint pinpoint;
    private FusedLocalizer fusedLocalizer;
    private Limelight limelight;

    private Side side;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        intake = new Intake();
        stopper = new Stopper();
        turret = new TurretV2(voltageSensor);
        pinpoint = new Pinpoint(new Transform(0, 0, 0));/*CrossModeStorage.position);*/
        limelight = new Limelight();
        limelight.localizationPipeline();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance);
        register(voltageSensor, gamepad, intake, stopper, turret, pinpoint, limelight, fusedLocalizer);
    }

    Stopwatch s;
    double rpm = 0;
    double hood = 0;

    @Override
    public void onStart() {
        s = new Stopwatch();
        schedule(
                new Repeat(() -> {
                    double dt = s.getDt();
                    rpm += gamepad.getRightX() * dt * 300;
                    hood += gamepad.getLeftX() * dt * 0.2;
                    turret.setHoodAngle(hood);
                    turret.setFlywheelSpeed(rpm);
                    intake.spin(gamepad.getRightTrigger());
                    stopper.spin(gamepad.getRightTrigger() - gamepad.getLeftTrigger() * 2);
                    telemetry.addData("RPM", rpm);
                    telemetry.addData("hood", hood);
                    telemetry.addData("distance", fusedLocalizer.getPosition().lateralDistance(new Transform(Constants.GOAL_X, Constants.GOAL_Y)));
                })
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Current RPM:", turret.getFlywheelSpeed());
        telemetry.addData("Distance to goal:", pinpoint.getPosition().lateralDistance(new Transform(Constants.GOAL_X, CrossModeStorage.side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0)));

        telemetry.addData("X", fusedLocalizer.getPosition().getX());
        telemetry.addData("Y", fusedLocalizer.getPosition().getY());
        telemetry.addData("Theta", fusedLocalizer.getPosition().getTheta());

        telemetry.addData("X Var", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Y Var", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Theta Var", fusedLocalizer.getCovariance().getEntry(2, 2));

        CrossModeStorage.position = fusedLocalizer.getPosition();

        telemetry.addData("Intake full", intake.isFull());

        telemetry.addData("Pinpoint X", pinpoint.getPosition().getX());
        telemetry.addData("Pinpoint Y", pinpoint.getPosition().getY());
        telemetry.addData("Pinpoint Theta", pinpoint.getPosition().getTheta());

        if (gamepad.getDpadRightPressedToggle()) {
            telemetry.addLine("Limelight DISABLED.");
            fusedLocalizer.disableLimelight();
        } else {
            fusedLocalizer.enableLimelight();
        }

        if (gamepad.getDpadUpJustPressed()) {
            fusedLocalizer.resetCovariance();
        }
    }
}
