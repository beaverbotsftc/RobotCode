package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.NoOp;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDriveControl;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretControl;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.List;
import java.util.function.DoubleUnaryOperator;

@TeleOp
public class TheTeleOpOfTheRobot extends CommandOpMode {
    public static int offset = 0;

    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;

    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    private GamepadEx gamepad;

    public void onInit() {
        setLynxUpdateFrequency(new int[] {1, 1, 1}, new int[] {0, 0, 0});
        setTelemetryUpdateFrequency(11, 0); // Should be coprime with everything else

        voltageSensor = new VoltageSensor(10, 1);
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new Intake(10, 2, 7);
        turret = new Turret(voltageSensor);

        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        localizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance.scalarMultiply(3));

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, pinpoint, limelight, localizer, gamepad);
    }

    public void periodicInit() {
        if (gamepad1.x) CrossModeStorage.side = Side.BLUE;
        if (gamepad1.b) CrossModeStorage.side = Side.RED;
        addData("Side", CrossModeStorage.side);
    }

    public void onStart() {
        register(drivetrain, intake, turret);

        Stopwatch stopwatch = new Stopwatch();
        schedule(
                new SwerveDriveControl((SwerveDrivetrain) drivetrain, localizer, gamepad,
                        CrossModeStorage.side == Side.RED
                                ? new DoubleUnaryOperator[]{ x -> x, y -> y, theta -> theta }
                                : new DoubleUnaryOperator[]{ x -> -x, y -> -y, theta -> theta }
                ),
                new Repeat(() -> intake.intake((gamepad.getRightTrigger() - gamepad.getLeftTrigger()) * (gamepad.getRightBumper() ? 0.75 : 1))),
                new Repeat(() -> intake.transfer(gamepad.getRightBumper() && turret.isFacingCorrectly())),
                new TurretControl(turret, localizer,
                        CrossModeStorage.side == Side.RED
                                ? new DoubleUnaryOperator[]{x -> x, y -> y, theta -> theta}
                                : new DoubleUnaryOperator[]{x -> x, y -> -y, theta -> -theta}
                )
        );

        schedule(new Repeat(() -> addData("dt", stopwatch.getDt())));
    }

    public void periodic() {
        addData("Position", localizer.getPosition());
        addData("Covariance X", localizer.getCovariance().getEntry(0, 0));
        addData("Covariance Y", localizer.getCovariance().getEntry(1, 1));
        addData("Covariance Theta", localizer.getCovariance().getEntry(2, 2));

        if (gamepad.getDpadUpJustPressed()) offset += 25;
        if (gamepad.getDpadDownJustPressed()) offset -= 25;
        addData("Offset", offset);
    }
}
