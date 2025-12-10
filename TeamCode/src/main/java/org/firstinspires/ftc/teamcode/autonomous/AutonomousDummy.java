package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Parallel;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Stopwatch;
import org.beaverbots.BeaverCommand.util.Wait;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.beavertracking.HolonomicFollowPath;
import org.beaverbots.beavertracking.PIDF;
import org.beaverbots.beavertracking.PIDFAxis;
import org.beaverbots.beavertracking.Path;
import org.beaverbots.beavertracking.PathBuilder;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousDummy extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Pinpoint pinpoint;
    private Limelight limelight;
    private Localizer fusedLocalizer;

    private final Side side = Side.RED;
    private Motif motif;

    private boolean motifScanning = false;

    private Stopwatch stopwatch = new Stopwatch();

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(0, 0, 0));

        register(gamepad, pinpoint, limelight, fusedLocalizer);
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
            pinpoint.setPosition(fusedLocalizer.getPosition());
            unregister(fusedLocalizer);
            limelight.obeliskPipeline();
            motifScanning = true;
        }

        if (!motifScanning) {
            telemetry.addData("Position:", fusedLocalizer.getPosition().toString());
        }
    }

    @Override
    public void onStart() {
        stopwatch.reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Position:", pinpoint.getPosition().toString());
        CrossModeStorage.position = pinpoint.getPosition();
    }
}
