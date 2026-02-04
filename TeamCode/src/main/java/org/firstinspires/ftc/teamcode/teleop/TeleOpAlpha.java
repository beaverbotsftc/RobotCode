package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.commands.AimWhileDriving;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.IntakeControlV2;
import org.firstinspires.ftc.teamcode.commands.ShooterControl2;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpAlpha extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private ColorSensor colorSensor;
    private FusedLocalizer fusedLocalizer;
    private Limelight limelight;
    private Led led;

    private Side side;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        stopper = new Stopper();
        shooter = new Shooter(voltageSensor);
        pinpoint = new Pinpoint(CrossModeStorage.position);
        colorSensor = new ColorSensor();
        limelight = new Limelight();
        limelight.localizationPipeline();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance);
        led = new Led();
        register(voltageSensor, gamepad, drivetrain, intake, stopper, shooter, pinpoint, colorSensor, led, limelight, fusedLocalizer);

        side = CrossModeStorage.side;
    }

    public void periodicInit() {
        if (gamepad.getCrossJustPressed()) {
            side = side == Side.RED ? Side.BLUE : Side.RED;
            CrossModeStorage.side = side;
        }
        telemetry.addData("Side", side);
    }

    Stopwatch s;

    @Override
    public void onStart() {
        s = new Stopwatch();
        schedule(
                new Repeat(() -> telemetry.addData("dt", s.getDt())),
                new Router(
                        new Selector(() -> gamepad.getRightStickPressedToggle()),
                        new Parallel(
                                new DrivetrainControl(drivetrain, gamepad),
                                new IntakeControlV2(intake, stopper, gamepad)
                        ),
                        new Parallel(
                                new AimWhileDriving(fusedLocalizer, drivetrain, side, gamepad),
                                new ShooterControl2(shooter, intake, stopper, fusedLocalizer, side, gamepad)
                        )
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Current RPM:", shooter.getVelocity());
        telemetry.addData("Distance to goal:", pinpoint.getPosition().lateralDistance(new Transform(Constants.GOAL_X, CrossModeStorage.side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0)));

        telemetry.addData("X", fusedLocalizer.getPosition().getX());
        telemetry.addData("Y", fusedLocalizer.getPosition().getY());
        telemetry.addData("Theta", fusedLocalizer.getPosition().getTheta());

        telemetry.addData("X Var", fusedLocalizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Y Var", fusedLocalizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Theta Var", fusedLocalizer.getCovariance().getEntry(2, 2));

        CrossModeStorage.position = fusedLocalizer.getPosition();

        telemetry.addData("Right trigger", gamepad.getRightTrigger());

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
