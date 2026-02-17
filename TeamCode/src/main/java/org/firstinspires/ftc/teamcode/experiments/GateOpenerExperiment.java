package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.GateOpener;

@Autonomous(group = "Experiments")
public class GateOpenerExperiment extends CommandRuntimeOpMode {
    GateOpener go;
    Gamepad gamepad;

    public void onInit() {
        go = new GateOpener();
        gamepad = new Gamepad(gamepad1);
        register(gamepad, go);
    }

    public void periodic() {
        if (!gamepad.getRightBumper())
            go.close();
        else
            go.open();
    }
}
