package org.firstinspires.ftc.teamcode.experiments;

import com.bylazar.field.PanelsField;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

@TeleOp
public class MecanumDrivetrainExperiment extends CommandOpMode {
    private MecanumDrivetrain drivetrain;

    @Override
    public void onInit() {
        this.drivetrain = new MecanumDrivetrain();
        register(drivetrain);
    }

    @Override
    public void periodic() {
        drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), gamepad.getRightX()));
    }
}
