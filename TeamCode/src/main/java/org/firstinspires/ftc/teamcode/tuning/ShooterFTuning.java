package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.optimizedhardware.OptimizedMotor;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous(group = "tuning")
public class ShooterFTuning extends LinearOpMode {
    public void runOpMode() {
        HardwareManager.init(hardwareMap);
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "left shooter");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "right shooter");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VoltageSensor voltageSensor = new VoltageSensor();

        waitForStart();
        while (opModeIsActive()) {
            voltageSensor.periodic();
            telemetry.addData("RPM per V",
                    Math.max(-shooterLeft.getVelocity(), shooterRight.getVelocity()) / 26 * 60 / voltageSensor.getVoltage()// It's reversed.
            );
            shooterLeft.setPower(0.5);
            shooterLeft.setPower(0.5);
            telemetry.update();
        }
    }
}
