package org.firstinspires.ftc.teamcode.experiments;

import android.util.Pair;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.concurrent.atomic.AtomicInteger;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(group = "Experiments")
public class LimelightExperiment2 extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Limelight limelight;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        limelight = new Limelight();
        limelight.localizationPipeline();

        register(gamepad);
    }

    @Override
    public void onStart() {
        AtomicInteger i = new AtomicInteger();
        schedule(new Repeat(() -> {
            Pair<Limelight.LimelightLocalization, Double> result = limelight.getEstimatedPosition();
            if (result == null)
                i.getAndIncrement();
            telemetry.addData("Robot pose", limelight.getEstimatedPosition());
            telemetry.addData("i", i);
        }));
    }
}
