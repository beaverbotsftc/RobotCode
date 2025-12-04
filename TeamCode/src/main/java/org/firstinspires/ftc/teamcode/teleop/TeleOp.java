package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.router.Router;
import org.beaverbots.BeaverCommand.util.router.Selector;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.IntakeControl;
import org.firstinspires.ftc.teamcode.commands.Resist;
import org.firstinspires.ftc.teamcode.commands.ShooterControl;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Stopper stopper;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private ShooterControl shooterControl;
    private ColorSensor colorSensor;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        stopper = new Stopper();
        shooter = new Shooter();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        colorSensor = new ColorSensor();
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, intake, stopper, shooter, pinpoint, limelight, colorSensor);
        shooterControl = new ShooterControl(shooter, gamepad);
        schedule(new Router(
                        new Selector(() -> gamepad.getLeftStickPressed()),
                        new DrivetrainControl(drivetrain, gamepad),
                        new Resist(pinpoint, drivetrain, Side.RED)),
                new IntakeControl(intake, stopper, colorSensor, gamepad), shooterControl);
        limelight.goalPipeline();
    }

    @Override
    public void periodic() {
        telemetry.addData("Shoot RPM:", shooterControl.getShootRpm());
        telemetry.addData("Current RPM:", shooterControl.getCurrentRPM());
    }
}
