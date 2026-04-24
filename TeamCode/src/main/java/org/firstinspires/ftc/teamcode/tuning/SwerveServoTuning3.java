package org.firstinspires.ftc.teamcode.tuning;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.HardwareManager;
import org.beaverbots.beaver.command.premade.Cycle;
import org.beaverbots.beaver.command.premade.Instant;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.command.premade.RunUntil;
import org.beaverbots.beaver.command.premade.Sequential;
import org.beaverbots.beaver.command.premade.WaitUntil;
import org.beaverbots.beaver.optimize.BayesianOptimizer;
import org.beaverbots.beaver.optimize.kernels.ARDRBFKernel;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;

@Autonomous(group = "tuning")
public class SwerveServoTuning3 extends CommandOpMode {
    private CRServo servo;
    private AnalogInput encoder;

    Stopwatch stopwatch = new Stopwatch();

    PIDFAxis pidf = new PIDFAxis(
            new PIDFAxis.K(
                    0.6, 0.0, 0.1, new double[]{0}, 1, 1, 0.9731, 988, 0.1
            )
    );

    public void onInit() {
        servo = HardwareManager.claim(CRServo.class, "back right servo");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder = HardwareManager.claim(AnalogInput.class, "back right servo encoder");
    }

    private double getAngle() {
        return encoder.getVoltage() / encoder.getMaxVoltage() * 2 * Math.PI;
    }

    @Override
    public void onStart() {
        schedule(
                new Repeat(() -> {
                    double dt = stopwatch.getDt();
                    double currentAngle = getAngle();
                    double error = currentAngle - gamepad1.left_stick_y - Math.PI;

                    double power = Math.max(-1, Math.min(pidf.update(error, new double[]{0.0}, dt), 1));

                    servo.setPower(power);
                })
        );
    }

    @Override
    public void periodic() {
    }
}