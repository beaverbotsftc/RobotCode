package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathAxis;
import org.beaverbots.beavertracking.PathBuilder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private Localizer localizer;

    private final Side side = Side.RED;
    private Motif motif;

    private boolean motifScanning = false;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        localizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(gamepad, drivetrain, intake, shooter, pinpoint, limelight, localizer);
        limelight.goalPipeline();
    }

    @Override
    public void periodicInit() {
        if (motifScanning) {
            Motif result = limelight.getMotif(side);
            if (result != null) motif = result;
            telemetry.addData("Limelight now:", result);
            telemetry.addData("Motif:", motif);
            telemetry.addLine(limelight.getStatus().toString());
        } else if (gamepad.getDpadUpJustPressed()) {
            unregister(localizer);
            limelight.obeliskPipeline();
            motifScanning = true;
        }

        if(!motifScanning) {
            telemetry.addData("Position:", localizer.getPosition().toString());
        }
    }

    @Override
    public void onStart() {
        //limelight.goalPipeline();
        //register(localizer);

        Pair<Path, Command> auto = new PathBuilder(localizer.getPositionAsList())
                .moveTo(new DrivetrainState(83.4, 58.7, -0.68).toList(), 10)
                .waitFor(20)
                .build(null);
        Command followPathCommand = new HolonomicFollowPath(
                auto.first,
                new PIDF(List.of(
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPX, Constants.pidIX, Constants.pidDX, 1, 6, 48, Constants.pidTauX)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPY, Constants.pidIY, Constants.pidDY, 1, 6, 48, Constants.pidTauY)),
                        new PIDFAxis(new PIDFAxis.K(Constants.pidPTheta, Constants.pidITheta, Constants.pidDTheta, 1, 6, 48, Constants.pidTauTheta)))),
                localizer, drivetrain);
        schedule(new Sequential(
                followPathCommand,
                new Instant(() -> drivetrain.move(new DrivetrainState(0, 0, 0)))
        ));
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", localizer.getPosition().toString());
    }
}
