package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.ControllerMove;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@TeleOp(group = "Experiments")
public class MecanumDrivetrainExperiment extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private MecanumDrivetrain drivetrain;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain(0.4);
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain);
        schedule(new ControllerMove(drivetrain, gamepad));
    }
}
