package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.commands.DrivetrainControl;
import org.firstinspires.ftc.teamcode.commands.ShooterMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@Autonomous
public class ControllableResistExperiment extends CommandRuntimeOpMode {
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;

    private boolean resist = false;

    private Command runningMovementCommand;

    @Override
    public void onInit() {
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain, pinpoint);
        runningMovementCommand = new DrivetrainControl(drivetrain, gamepad);
        schedule(runningMovementCommand);
    }

    @Override
    public void periodic() {
        telemetry.addLine(resist ? "Resisting" : "Moving");
        if (gamepad.getCrossJustPressed()) {
            resist = !resist;

            cancel(runningMovementCommand);
            if (resist) {
                schedule(new ShooterMode(pinpoint, drivetrain, Side.RED));
            } else {
                schedule(new DrivetrainControl(drivetrain, gamepad));
            }
        }
    }
}
