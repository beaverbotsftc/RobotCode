package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@TeleOp
public class FieldCentricSwerveDrivingExperiment extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private SwerveDrivetrain drivetrain;
    private GamepadEx gamepad;
    private DcMotorEx transfer1;
    private DcMotorEx transfer2;
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo stopper;
    private Pinpoint pinpoint;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);
        gamepad = new GamepadEx(gamepad1);

        transfer1 = HardwareManager.claim(DcMotorEx.class, "transfer1");
        transfer2 = HardwareManager.claim(DcMotorEx.class, "transfer2");
        shooterLeft = HardwareManager.claim(DcMotorEx.class, "left shooter");
        shooterRight = HardwareManager.claim(DcMotorEx.class, "right shooter");

        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopper = HardwareManager.claim("stopper");

        pinpoint = new Pinpoint(new Transform(0, 0, 0));

        register(voltageSensor, pinpoint, drivetrain, gamepad);
    }

    public void onStart() {
        schedule(new Repeat(() -> {
            drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()).toLocalVelocity(pinpoint.getPosition()));
        }));
    }

    public void periodic() {
        transfer1.setPower(gamepad.getRightTrigger() - gamepad.getLeftTrigger());
        transfer2.setPower(gamepad.getRightTrigger() - gamepad.getLeftTrigger());

        shooterLeft.setPower(0.5);
        shooterRight.setPower(0.5);

        stopper.setPosition(gamepad.getRightBumper() ? 0.6 : 0.37);
    }
}
