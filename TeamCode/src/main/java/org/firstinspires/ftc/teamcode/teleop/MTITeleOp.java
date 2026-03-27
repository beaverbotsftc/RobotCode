package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Defer;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.NoOp;
import org.beaverbots.beaver.command.premade.Parallel;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.StartWith;
import org.beaverbots.beaver.command.premade.WaitUntil;
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
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;
import java.util.function.ToDoubleFunction;

@TeleOp
public class MTITeleOp extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private VoltageSensor voltageSensor;
    private Turret turret;

    private FusedLocalizer localizer;

    private GamepadEx gamepad;

    private ToDoubleFunction<Pair<List<Double>, List<Double>>> usageRatio;


    public void onInit() {
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        voltageSensor = new VoltageSensor();
        turret = new Turret(voltageSensor);

        Pinpoint pinpoint = new Pinpoint(new Transform(0, 0, 0));
        Limelight limelight = new Limelight();

        localizer = new FusedLocalizer(
                pinpoint,
                limelight,
                new Transform(0, 0, 0)
        );

        gamepad = new GamepadEx(gamepad1);

        usageRatio = PathBuilder.createHolonomicUsage(1 / Constants.drivetrainPowerConversionFactorX, 1 / Constants.drivetrainPowerConversionFactorY, 1 / Constants.drivetrainPowerConversionFactorTheta);

        register(drivetrain, intake, turret, voltageSensor, pinpoint, limelight, localizer, gamepad);
    }

    public void periodicInit() {
        telemetry.addData("Position X", localizer.getPosition().getX());
        telemetry.addData("Position Y", localizer.getPosition().getY());
        telemetry.addData("Position Theta", localizer.getPosition().getTheta());
        telemetry.addData("Variance X", localizer.getCovariance().getEntry(0, 0));
        telemetry.addData("Variance Y", localizer.getCovariance().getEntry(1, 1));
        telemetry.addData("Variance Theta", localizer.getCovariance().getEntry(2, 2));
        telemetry.addData("RPM", turret.getVelocity());
    }

    public void periodic() {
        periodicInit();
    }

    public void onStart() {
        turret.shoot(2800);
        schedule(
                new Router(
                        new Selector(() -> gamepad.getX()),
                        new StartWith(
                                new Parallel(
                                        new Repeat(
                                                () -> drivetrain.move(
                                                        new Transform(
                                                                gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorX,
                                                                -gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorY,
                                                                -gamepad.getRightX() / Constants.drivetrainPowerConversionFactorTheta
                                                        )
                                                )
                                        ),
                                        new Repeat(() -> {
                                            double control = gamepad.getRightTrigger() - gamepad.getLeftTrigger();
                                            if (control == 0)
                                                intake.stop();
                                            else
                                                intake.intake(control > 0);
                                        })
                                ),
                                () -> intake.transfer(false)
                        ),
                        new StartWith(
                                new Defer(() -> {
                                        Transform shootLocation = new Transform(-24, 26);
                                        Transform goalLocation = new Transform(-72, 72);
                                        double angle = Geometry.unnormalizeAngle(localizer.getPosition().angleTo(shootLocation), localizer.getPosition().getTheta());
                                        boolean invertAngle = Math.abs(localizer.getPosition().getTheta() - angle) >= Math.PI;
                                        Transform shootPose = new Transform(shootLocation.getX(), shootLocation.getY(), Geometry.unnormalizeAngle(invertAngle ? angle + Math.PI : angle, localizer.getPosition().getTheta()));
                                        Path pathThere = new PathBuilder(localizer.getPositionAsList())
                                                .bezierTo(
                                                        localizer.getPosition().lateral().add(shootPose.angular()).blend(shootPose, 1.0 / 3.0).toList(),
                                                        localizer.getPosition().lateral().add(shootPose.angular()).blend(shootPose, 2.0 / 3.0).toList(),
                                                        shootPose.toList(),
                                                        0.2,
                                                        1
                                                )
                                                .build()
                                                .first;
                                        Pair<Path, Path> timedPathThere = new PathBuilder(pathThere)
                                                .retime(usageRatio, 1, 50)
                                                .build();
                                        Pair<Path, Path> timedPathBack = new PathBuilder(pathThere)
                                                .reverse()
                                                .retime(usageRatio, 1, 50)
                                                .build();
                                        return new Sequential(
                                                new RunUntil(
                                                        new Sequential(
                                                                followPathTemplate(timedPathThere.first, 1),
                                                                new RunUntil(
                                                                        new Sequential(
                                                                                new WaitUntil(() -> gamepad.getA()),
                                                                                new Instant(() -> intake.intake(true)),
                                                                                new WaitUntil(() -> !gamepad.getA()),
                                                                                new Instant(() -> intake.stop())
                                                                        ),
                                                                        followPathTemplate(timedPathThere.second, 0.3)
                                                                )
                                                        ),
                                                        new Repeat(() -> turret.turn(localizer.getPosition().angleTo(goalLocation.subtract(localizer.getVelocity().scale(0.7))) - localizer.getPosition().getTheta()))
                                                ),
                                                followPathTemplate(timedPathBack.first, 1)
                                        );
                                    }
                                ),
                                () -> {
                                    intake.transfer(true);
                                    intake.stop();
                                }
                        )
                )
        );
    }


    private Command followPathTemplate(Path path, double multiplier) {
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
