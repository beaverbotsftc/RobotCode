package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.HardwareManager;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.ControllerMove;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

@TeleOp(group = "Experiments")
public class MecanumDrivetrainTest extends CommandRuntimeOpMode {
    Gamepad gamepad;
    MecanumDrivetrain drivetrain;

    @Override
    public void onInit() {
        DcMotorEx frontLeft = HardwareManager.claim("left front");
        DcMotorEx frontRight = HardwareManager.claim("right front");
        DcMotorEx backLeft = HardwareManager.claim("left back");
        DcMotorEx backRight = HardwareManager.claim("right back");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain(frontLeft, frontRight, backLeft, backRight);
        drivetrain.setMaxPower(0.7);
    }

    @Override
    public void onStart() {
        register(gamepad, drivetrain);
        schedule(new ControllerMove(drivetrain, gamepad));
    }
}
