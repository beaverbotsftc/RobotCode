package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.ControllerMove;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeMove;

@TeleOp(group = "Experiments")
public class IntegrationExperiment extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private MecanumDrivetrain drivetrain;
    private Intake intake;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain(0.8);
        intake = new Intake(0.1);
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, intake);
        schedule(new ControllerMove(drivetrain, gamepad), new IntakeMove(intake, gamepad));
    }
}
