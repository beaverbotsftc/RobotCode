package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;

@Autonomous(group = "tuning")
public class DrivetrainEqualityTuning extends CommandOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void onInit() {
        frontLeft = HardwareManager.get(DcMotorEx.class, "front left drive");
        frontRight = HardwareManager.get(DcMotorEx.class, "front right drive");
        backLeft = HardwareManager.get(DcMotorEx.class, "back left drive");
        backRight = HardwareManager.get(DcMotorEx.class, "back right drive");
    }

    @Override
    public void periodic() {
        frontLeft.setPower(gamepad1.left_stick_x);
        frontRight.setPower(gamepad1.left_stick_x);
        backLeft.setPower(gamepad1.left_stick_x);
        backRight.setPower(gamepad1.left_stick_x);

        addData("front left", frontLeft.getVelocity());
        addData("front right", frontRight.getVelocity());
        addData("back left", backLeft.getVelocity());
        addData("back right", backRight.getVelocity());
    }
}
