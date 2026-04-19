package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous
public class TurretExperiment extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Turret turret;
    private GamepadEx gamepad;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        turret = new Turret(voltageSensor);
        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, turret, gamepad);
    }

    double angle = 0;
    double rpm = 0;

    @Override
    public void periodic() {
        angle += gamepad.getLeftX() * 0.005;
        rpm += gamepad.getRightX();

        turret.turn(angle);
        turret.shoot(rpm);

        telemetry.addData("angle", angle);
        telemetry.addData("rpm", rpm);
    }
}
