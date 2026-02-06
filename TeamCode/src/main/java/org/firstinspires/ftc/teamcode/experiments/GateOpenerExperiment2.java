package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.GateOpener;

@Autonomous(group = "Experiments")
public class GateOpenerExperiment2 extends CommandRuntimeOpMode {
    GateOpener go;
    public void onInit() {
        go = new GateOpener();
    }

    public void periodic() {
        if (gamepad1.a) go.open();
        else go.close();
    }
}
