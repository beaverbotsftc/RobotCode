package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class TurretExperiment extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Turret turret;
    private GamepadEx gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        turret = new Turret(voltageSensor);
        gamepad = new GamepadEx(gamepad1);

        register(turret, gamepad);
    }

    public void periodic() {
        turret.turn(-2 * gamepad.getRightX() + Math.PI);
    }
}
