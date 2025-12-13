package org.firstinspires.ftc.teamcode.experiments;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.BeaverCommand.CommandRuntimeOpMode;
import org.beaverbots.BeaverCommand.util.Instant;
import org.beaverbots.BeaverCommand.util.Repeat;
import org.beaverbots.BeaverCommand.util.RunUntil;
import org.beaverbots.BeaverCommand.util.Sequential;
import org.beaverbots.BeaverCommand.util.Wait;
import org.beaverbots.BeaverCommand.util.WaitUntil;
import org.beaverbots.BeaverOptimize.BayesianOptimizer;
import org.beaverbots.BeaverOptimize.util.RBFKernel;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class ShooterRPMExperiment extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Shooter shooter;
    private Gamepad gamepad;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        shooter = new Shooter(voltageSensor);
        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(voltageSensor, shooter, gamepad);
        schedule(new Repeat(() -> telemetry.addData("RPM", shooter.getVelocity())));
    }

    @Override
    public void periodic() {
        if (gamepad.getDpadUpJustPressed())
            shooter.spin(4000);
        if (gamepad.getDpadRightJustPressed())
            shooter.spin(3000);
        if (gamepad.getDpadDownJustPressed())
            shooter.spin(2000);
        if (gamepad.getDpadLeftJustPressed())
            shooter.spin(1000);
        if (gamepad.getCircleJustPressed())
            shooter.spin(0);
    }
}
