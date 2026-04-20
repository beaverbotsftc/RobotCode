package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDriveControl;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretControl;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.function.DoubleUnaryOperator;

@TeleOp
public class TheTeleOpOfTheRobot extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private IntakeAndTransfer intake;
    private Turret turret;

    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    private GamepadEx gamepad;

    public void onInit() {
        setLynxUpdateFrequency(new int[] {1, 1, 999}, new int[] {0, 0, 0});
        setTelemetryUpdateFrequency(10, 0);

        voltageSensor = new VoltageSensor(10, 1);
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new IntakeAndTransfer(10, 2, 7);
        turret = new Turret(voltageSensor);

        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        localizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, pinpoint, limelight, localizer, drivetrain, intake, turret, gamepad);
    }

    public void onStart() {
        Stopwatch stopwatch = new Stopwatch();
        schedule(
                new SwerveDriveControl((SwerveDrivetrain) drivetrain, localizer, gamepad, new DoubleUnaryOperator[]{ x -> x, y -> y, theta -> theta }),
                new Repeat(() -> intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger())),
                new Repeat(() -> intake.transfer(gamepad.getRightBumper())),
                new Router(new Selector(() -> gamepad.getBPressedToggle()),
                        new TurretControl(turret, localizer, new DoubleUnaryOperator[]{x -> x, y -> y, theta -> theta}),
                        new Instant(() -> turret.shoot(0))
                )
        );

        schedule(new Repeat(() -> addData("dt", stopwatch.getDt())));
    }

    public void periodic() {
        addData("Position", localizer.getPosition());
        addData("Covariance X", localizer.getCovariance().getEntry(0, 0));
        addData("Covariance Y", localizer.getCovariance().getEntry(1, 1));
        addData("Covariance Theta", localizer.getCovariance().getEntry(2, 2));
    }
}
