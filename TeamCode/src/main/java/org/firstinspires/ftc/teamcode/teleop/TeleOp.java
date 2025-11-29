package org.firstinspires.ftc.teamcode.teleop;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.router.Router;
import org.beaverbots.BeaverCommand.util.router.Selector;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.IntakeControl;
import org.firstinspires.ftc.teamcode.commands.Resist;
import org.firstinspires.ftc.teamcode.commands.ShooterControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Pinpoint pinpoint;

    private ShooterControl shooterControl;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, intake, shooter, pinpoint);
        shooterControl = new ShooterControl(shooter, gamepad);
        schedule(new Router(
                        new Selector(() -> gamepad.getLeftStickPressed()),
                        new DrivetrainControl(drivetrain, gamepad),
                        new Resist(pinpoint, drivetrain)),
                new IntakeControl(intake, gamepad), shooterControl);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shoot RPM:", shooterControl.getShootRpm());
    }
}
