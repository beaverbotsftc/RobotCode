package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
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
public class TeleOpBare extends CommandRuntimeOpMode {
    private static final Logger log = LoggerFactory.getLogger(TeleOpBare.class);
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
                new DrivetrainControl(drivetrain, gamepad),
                new Repeat(() -> {
                    if (gamepad.getRightTrigger() != 0) {
                        intake.spin(1);
                        stopper.spin(-1);
                    } else if (gamepad.getRightBumper()) {
                        intake.spin(1);
                        stopper.spin(1);
                    } else {
                        intake.spin(0);
                        stopper.spin(0);
                    }

                    if (gamepad.getTrianglePressedToggle()) { // Wall
                        led.setRightHue(0.2); // Yellow = Wall
                        if (gamepad.getLeftBumperPressedToggle()) { // Far-ish
                            shooter.setFlywheelSpeed(Shooter.getSettingsAtDistanceWall(65).first);
                            shooter.setHoodAngle(Shooter.getSettingsAtDistanceWall(65).second);
                        } else { // Close-ish
                            shooter.setFlywheelSpeed(Shooter.getSettingsAtDistanceWall(43).first);
                            shooter.setHoodAngle(Shooter.getSettingsAtDistanceWall(43).second);
                        }
                    } else {
                        led.setRightHue(1); // Purple = normal
                        if (gamepad.getLeftBumperPressedToggle()) { // Far-ish
                            shooter.setFlywheelSpeed(Shooter.getSettingsAtDistanceNonWall(72).first);
                            shooter.setHoodAngle(Shooter.getSettingsAtDistanceNonWall(72).second);
                        } else { // Close-ish
                            shooter.setFlywheelSpeed(Shooter.getSettingsAtDistanceNonWall(61).first);
                            shooter.setHoodAngle(Shooter.getSettingsAtDistanceNonWall(61).second);
                        }
                    }
                })
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