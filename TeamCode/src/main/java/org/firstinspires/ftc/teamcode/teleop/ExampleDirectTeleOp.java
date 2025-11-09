package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.commands.DirectControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@TeleOp
public class ExampleDirectTeleOp extends CommandRuntimeOpMode {
    Gamepad gamepad;
    Drivetrain drivetrain;
    Intake intake;
    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain(0.3);
        intake = new Intake(0.4);
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, intake);
        schedule(new DirectControl(gamepad, drivetrain, intake));
    }
}
