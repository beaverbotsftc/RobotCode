package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@Autonomous(group = "Experiments")
public class PinpointExperiment extends CommandRuntimeOpMode {
    private Pinpoint pinpoint;

    @Override
    public void onInit() {
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(pinpoint);
    }

    @Override
    public void periodic() {
        telemetry.addData("X Position", pinpoint.getPosition().getX());
        telemetry.addData("Y Position", pinpoint.getPosition().getY());
        telemetry.addData("Theta Position", pinpoint.getPosition().getTheta() * 180 / Math.PI);
        telemetry.update();
    }
}
