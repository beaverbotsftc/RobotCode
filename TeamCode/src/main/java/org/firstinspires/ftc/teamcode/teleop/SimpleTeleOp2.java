package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.linear.RealMatrix;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@TeleOp
public class SimpleTeleOp2 extends CommandRuntimeOpMode {
    Servo s;

    @Override
    public void onInit() {
        s = hardwareMap.get(Servo.class, "hood");
    }

    double a = 0;

    @Override
    public void periodic() {
        a += gamepad1.left_stick_x / 1000;
        telemetry.addData("a",a);
        s.setPosition(a);
    }
}