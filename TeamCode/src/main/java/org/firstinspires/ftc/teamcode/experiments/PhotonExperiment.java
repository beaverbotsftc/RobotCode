package org.firstinspires.ftc.teamcode.experiments;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.teamcode.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.beaver.command.HardwareManager;
import org.firstinspires.ftc.teamcode.beaver.util.Stopwatch;

@TeleOp
public class PhotonExperiment extends CommandOpMode {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;

    private Stopwatch stopwatch;

    @Override
    public void onInit() {
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        motor1 = HardwareManager.claim("front left drive");
        motor2 = HardwareManager.claim("front right drive");
        motor3 = HardwareManager.claim("back left drive");
        motor4 = HardwareManager.claim("back right drive");

        stopwatch = new Stopwatch();

        telemetryUpdateWait = 100;
    }

    @Override
    public void periodic() {
        motor1.setPower(Math.random() * 0.01);
        motor2.setPower(Math.random() * 0.01);
        motor3.setPower(Math.random() * 0.01);
        motor4.setPower(Math.random() * 0.01);

        addData("dt", stopwatch.getDt() * 1000);
    }
}
