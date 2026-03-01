package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Defer;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.Wait;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.beaverbots.beaver.pathing.commands.RamseteFollowPath;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.commands.AimWhileDriving;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.GoToBase;
import org.firstinspires.ftc.teamcode.commands.IntakeControlV2;
import org.firstinspires.ftc.teamcode.commands.Resist;
import org.firstinspires.ftc.teamcode.commands.ShooterControl2;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Led2;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpAlpha extends CommandRuntimeOpMode {
    private static final Logger log = LoggerFactory.getLogger(TeleOpAlpha.class);
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
    private Led2 led;

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
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance.scalarMultiply(5));
        led = new Led2();
        register(voltageSensor, gamepad, drivetrain, intake, stopper, shooter, pinpoint, colorSensor, led, limelight, fusedLocalizer);

        side = CrossModeStorage.side;
    }

    public void periodicInit() {
        if (gamepad.getSquareJustPressed()) {
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
                        new Selector(() -> {
                            if (gamepad.getGuide())
                                return 5;
                            if (gamepad.getLeftStickPressed() && gamepad.getRightStickPressedToggle())
                                return 4;
                            if (gamepad.getLeftStickPressed())
                                return 3;
                            if (gamepad.getRightStickPressedToggle() && gamepad.getTrianglePressedToggle()) // On wall
                                return 2;
                            if (gamepad.getRightStickPressedToggle())
                                return 1;
                            return 0;
                        }
                        ),
                        new Parallel(
                                new DrivetrainControl(drivetrain, gamepad),
                                new IntakeControlV2(intake, stopper, gamepad),
                                new Repeat(() -> {
                                    if (intake.isFull())
                                        led.setLeftHue(0.4); // Full = green
                                    else if (intake.isEmpty())
                                        led.setLeftHue(0); // Empty = red
                                    else
                                        led.turnOffLeft();

                                    if (gamepad.getTrianglePressedToggle())
                                        led.setRightHue(0.2); // Wall = yellow
                                    else
                                        led.setRightHue(1); // Normal = purple

                                    telemetry.addData("Mode", "Driving");
                                })
                        ),
                        new Parallel(
                                new AimWhileDriving(fusedLocalizer, drivetrain, side, gamepad, false),
                                new ShooterControl2(shooter, intake, stopper, fusedLocalizer, side, gamepad, false, true),
                                new Repeat(() -> {
                                    Transform goalPosition = new Transform(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0);

                                    if (intake.isEmpty())
                                        led.setLeftHue(0); // Empty = red
                                    else
                                        led.setLeftHue( // Closer to purple means better aim
                                                Math.max(0, 1 - Math.abs(fusedLocalizer.getPosition().relativeAngleTo(goalPosition)))
                                        );

                                    led.setRightHue(Math.max(0, 1 - Math.abs(shooter.getError() / 1000))); // Closer to purple means better RPM

                                    telemetry.addData("Mode", "Shooting - Normal");
                                })
                        ),
                        new Parallel(
                                new AimWhileDriving(fusedLocalizer, drivetrain, side, gamepad, true),
                                new ShooterControl2(shooter, intake, stopper, fusedLocalizer, side, gamepad, true, false),
                                new Repeat(() -> {
                                    led.turnOffLeft();

                                    led.setRightHue(1 - Math.abs(shooter.getError() / 1000)); // Closer to purple means better RPM

                                    telemetry.addData("Mode", "Shooting - Wall");
                                })
                        ),
                        new Parallel(
                                new Resist(fusedLocalizer, drivetrain, side, false),
                                new Repeat(() -> {
                                    led.turnOffLeft();
                                    led.turnOffRight();

                                    telemetry.addData("Mode", "Resisting w/o aiming");
                                })
                        ),
                        new Parallel(
                                new Resist(fusedLocalizer, drivetrain, side, true),
                                new ShooterControl2(shooter, intake, stopper, fusedLocalizer, side, gamepad, false, false),
                                new Repeat(() -> {
                                    led.turnOffLeft();
                                    led.turnOffRight();

                                    telemetry.addData("Mode", "Resisting w/ aiming");
                                })
                        ),
                        new Parallel(
                                new GoToBase(fusedLocalizer, drivetrain, side),
                                new Repeat(() -> {
                                    led.turnOffLeft();
                                    led.turnOffRight();

                                    telemetry.addData("Mode", "Going to base");
                                })
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