package org.firstinspires.ftc.teamcode.autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

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
