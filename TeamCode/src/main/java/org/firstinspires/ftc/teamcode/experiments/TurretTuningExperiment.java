package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Stopwatch;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class TurretTuningExperiment extends CommandRuntimeOpMode {
    private Stopwatch stopwatch;
    VoltageSensor voltageSensor;
    TurretV2 turret;
    Gamepad gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        turret = new TurretV2(voltageSensor);
        gamepad = new Gamepad(gamepad1);

        register(voltageSensor, turret, gamepad);
    }

    public void onStart() {
        stopwatch = new Stopwatch();
    }

    public void periodic() {
        double dt = stopwatch.getDt();

        turret.setTurretAngle(0.4 * dt * gamepad.getRightX() + turret.getTurretAngle());
        turret.setHoodAngle(0.2 * dt * gamepad.getLeftX() + turret.getHoodAngle());
        turret.spin(100 * dt * gamepad.getRightY() + turret.getDesiredRpm());

        telemetry.addData("Turret Angle", turret.getTurretAngle());
        telemetry.addData("Hood Angle", turret.getHoodAngle());
        telemetry.addData("RPM (Actual)", turret.getVelocity());
        telemetry.addData("RPM (Desired)", turret.getDesiredRpm());
    }
}
