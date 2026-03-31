package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Defer;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.router.Router;
import org.beaverbots.beaver.command.premade.router.Selector;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.path.Path;
import org.beaverbots.beaver.pathing.path.pathbuilder.PathBuilder;
import org.beaverbots.beaver.pidf.PIDF;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Geometry;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.CrossModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.turret.TrackTarget;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;
import java.util.function.ToDoubleFunction;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private Turret turret;
    private Intake intake;
    private GamepadEx gamepad;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer localizer;
    private ToDoubleFunction<Pair<List<Double>, List<Double>>> usageRatio;

    public void onInit() {
        setTelemetryUpdateFrequency(10);

        voltageSensor = new VoltageSensor();
        drivetrain = new MecanumDrivetrain();
        turret = new Turret(voltageSensor);
        intake = new Intake();

        gamepad = new GamepadEx(gamepad1);

        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight();
        limelight.localizationPipeline();

        localizer = new FusedLocalizer(pinpoint, limelight, CrossModeStorage.position, CrossModeStorage.covariance);

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(voltageSensor, drivetrain, turret, intake, gamepad, pinpoint, limelight, localizer);
    }

    public void periodicInit() {
        telemetry.addData("Position:", localizer.getPosition().toString());
        telemetry.addData("Variance X:", localizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Variance Y:", localizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Variance Theta:", localizer.getCovariance().getEntry(2, 2));
        telemetry.addData("Flywheel RPM:", turret.getVelocity());
    }

    public void onStart() {
        turret.shoot(2700);
        schedule(
                new Router(new Selector(() -> gamepad.getA()),
                        new Repeat(() -> drivetrain.move(new Transform(gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorX, -gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorY, -gamepad.getRightX() / Constants.drivetrainPowerConversionFactorTheta))),
                        new Defer(() -> {
                                        double angle = Geometry.normalizeAngle2(localizer.getPosition().getTheta());
                                        if (Math.abs(angle - 3 * Math.PI / 4) > Math.PI / 2) angle = 3 * Math.PI / 4;
                                        Pair<Path, Path> paths =
                                                new PathBuilder(localizer.getPosition().toList())
                                                        .linearTo(new Transform(-24, 24, Geometry.unnormalizeAngle(angle, localizer.getPosition().getTheta())).toList(), 0.2, 1)
                                                        .stop(0.3, 0.3)
                                                        .build();
                                        return new Sequential(
                                                followPath(paths.first, 1),
                                                followPath(paths.second, 0.6)
                                        );
                                }
                        )

                ),
                new TrackTarget(turret, localizer, new Transform(-72, 72)),
                new Repeat(() -> {
                    if (gamepad.getRightBumper()) intake.intake(true);
                    else intake.stop();
                }),
                new Repeat(() -> intake.transfer(gamepad.getX()))
        );
    }

    public void periodic() {
        periodicInit();
    }

    private Command followPath(Path path, double multiplier) {
        return new Sequential(
                new HolonomicFollowPath(
                        path,
                        new PIDF(List.of(
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPX * multiplier, Constants.pidIX * multiplier, Constants.pidDX * multiplier, 1, 6, 48, Constants.pidTauX, Constants.pidGammaX)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPY * multiplier, Constants.pidIY * multiplier, Constants.pidDY * multiplier, 1, 6, 48, Constants.pidTauY, Constants.pidGammaY)),
                                new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta * multiplier, Constants.pidITheta * multiplier, Constants.pidDTheta * multiplier, 1, 6, 48, Constants.pidTauTheta, Constants.pidGammaTheta)))),
                        localizer, drivetrain),
                new Instant(() -> drivetrain.move(new Transform(0, 0, 0)))
        );
    }
}
