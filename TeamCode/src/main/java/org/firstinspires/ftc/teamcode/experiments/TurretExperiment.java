package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous
public class TurretExperiment extends CommandRuntimeOpMode {
    private Turret turret;
    private Gamepad gamepad;

    public void onInit() {
        turret = new Turret();
        gamepad = new Gamepad(gamepad1);

        register(turret, gamepad);
    }

    public void periodic() {
        turret.turn(gamepad.getRightX() + Math.PI);
    }
}
