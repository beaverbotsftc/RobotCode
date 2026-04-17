package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.NoOp;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
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
import org.firstinspires.ftc.teamcode.subsystems.turret.TrackTarget;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
public class TheTeleOpOfTheRobot extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private IntakeAndTransfer intake;
    private Turret turret;

    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;

    private GamepadEx gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new IntakeAndTransfer();
        turret = new Turret(voltageSensor);

        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight(Limelight.Pipeline.LOCALIZATION_GOAL);
        localizer = new FusedLocalizer(pinpoint, limelight, new Transform(0, 0, 0));

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, pinpoint, limelight, localizer, drivetrain, intake, turret, gamepad);
    }

    double rpm = 0;

    public void onStart() {
        schedule(
                new SwerveDriveControl((SwerveDrivetrain) drivetrain, localizer, gamepad),
                new Repeat(() -> intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger())),
                new Repeat(() -> intake.transfer(gamepad.getRightBumper())),
                new TrackTarget(turret, localizer, new Transform(-72, 72))
        );
    }

    public void periodic() {
        telemetry.addData("RPM", rpm);
        rpm += gamepad.getDpadUpJustPressed() ? 100 : 0;
        rpm -= gamepad.getDpadDownJustPressed() ? 100 : 0;
        turret.shoot(rpm);

        getTelemetry().addData("Position", localizer.getPosition());
        getTelemetry().addData("Covariance X", localizer.getCovariance().getEntry(0, 0));
        getTelemetry().addData("Covariance Y", localizer.getCovariance().getEntry(1, 1));
        getTelemetry().addData("Covariance Theta", localizer.getCovariance().getEntry(2, 2));
    }
}
