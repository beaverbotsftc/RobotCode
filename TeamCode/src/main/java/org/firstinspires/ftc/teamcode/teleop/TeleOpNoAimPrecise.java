package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.IntakeControl;
import org.firstinspires.ftc.teamcode.commands.ShooterControlPrecise;
import org.firstinspires.ftc.teamcode.commands.ShooterMode;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpNoAimPrecise extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private ShooterControlPrecise shooterControl;
    private ColorSensor colorSensor;
    private Led led;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake(voltageSensor);
        stopper = new Stopper();
        shooter = new Shooter(voltageSensor);
        pinpoint = new Pinpoint(CrossModeStorage.position);
        colorSensor = new ColorSensor();
        led = new Led();
        register(voltageSensor, gamepad, drivetrain, intake, stopper, shooter, pinpoint, colorSensor, led);
    }

    @Override
    public void onStart() {
        shooterControl = new ShooterControlPrecise(shooter, gamepad);
        schedule(new Router(
                        new Selector(() -> gamepad.getLeftStickPressed()),
                        new DrivetrainControl(drivetrain, gamepad),
                        new ShooterMode(pinpoint, drivetrain, Side.RED, true)),
                new IntakeControl(intake, stopper, null, false, CrossModeStorage.side, colorSensor, led, gamepad), shooterControl);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shoot RPM:", shooterControl.getShootRpm());
        telemetry.addData("Current RPM:", shooter.getVelocity());
        telemetry.addData("Current Hood Position", shooter.getHood());
    }
}