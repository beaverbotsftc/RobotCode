package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.autonomous.CrossModeStorage;
import org.firstinspires.ftc.teamcode.commands.AimAndResist;
import org.firstinspires.ftc.teamcode.commands.AimWhileDriving;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.GoToBase;
import org.firstinspires.ftc.teamcode.commands.IntakeControl;
import org.firstinspires.ftc.teamcode.commands.ShooterMode;
import org.firstinspires.ftc.teamcode.commands.ShooterControl;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private ShooterControl shooterControl;
    private ColorSensor colorSensor;
    private FusedLocalizer fusedLocalizer;
    private Limelight limelight;
    private Led led;

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
        limelight.goalPipeline();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position);
        led = new Led();
        register(voltageSensor, gamepad, drivetrain, intake, stopper, shooter, pinpoint, colorSensor, led, limelight, fusedLocalizer);
    }

    @Override
    public void onStart() {
        shooterControl = new ShooterControl(shooter, pinpoint, CrossModeStorage.side, led, gamepad);
        schedule(
                /*new Router(
                        new Selector(() -> gamepad.getLeftStickPressed()),
                        new DrivetrainControl(drivetrain, gamepad),
                        new ShooterMode(pinpoint, drivetrain, CrossModeStorage.side, false)),
                new IntakeControl(intake, stopper, colorSensor, led, gamepad), shooterControl);
                        new ShooterMode(pinpoint, drivetrain, CrossModeStorage.side, false)
                ),*/
                new Router(
                        new Selector(() -> {
                                if (gamepad.getRightStickPressedToggle() && gamepad.getLeftStickPressed()) {
                                    return 1;
                                }
                                if (gamepad.getLeftStickPressed()) {
                                    return 2;
                                }
                                if (gamepad.getRightStickPressedToggle()) {
                                    return 3;
                                }
                                if (gamepad.getGuide()) {
                                    return 4;
                                }
                                return 0;
                            }
                        ),
                        new DrivetrainControl(drivetrain, gamepad),
                        new AimAndResist(pinpoint, drivetrain, CrossModeStorage.side, true),
                        new AimAndResist(pinpoint, drivetrain, CrossModeStorage.side, false),
                        new AimWhileDriving(pinpoint, drivetrain, CrossModeStorage.side, gamepad),
                        new GoToBase(pinpoint, drivetrain, CrossModeStorage.side)
                ),
                new IntakeControl(intake, stopper, colorSensor, led, gamepad), shooterControl);
    }

    @Override
    public void periodic() {

        telemetry.addData("Shoot RPM:", shooterControl.getShootRpm());
        telemetry.addData("Current RPM:", shooter.getVelocity());
        telemetry.addData("Fused position:", fusedLocalizer.getPosition());
        telemetry.addData("Distance to Goal:", shooterControl.getDistanceToTag());
        /*
        if (gamepad.getDpadUpJustPressed() && !a) {
            pinpoint.setPosition(fusedLocalizer.getPosition());
            a = true;
        }
         */
    }
}
