package org.firstinspires.ftc.teamcode.pathfollower2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.collections.Sensors;

import java.util.function.BiFunction;
import java.util.function.Function;

public class PathFollowerTuningWeights extends LinearOpMode {
    final double reinitTime = 1;

    final List<BiFunction<Motors, Sensors, Double>> lossFunctions = List.of((Motors motors, Sensors sensors) -> 0.0);

    List<double[]> weightPool = List.of(TuningConstants.weights);
    List<Double> loss = List.of(0.0);

    public void reset(Sensors sensors, Telemetry telemetry) {
        sensors.odometry.resetPosAndIMU();
        sleep((long) (1000 * reinitTime));
    }

    public double getLoss(double[] weights, Motors motors, Sensors sensors) {
        double accumulator = 0.0;
        for (int i = 0; i < lossFunctions.length(); i++) {
            accumulator += lossFunctions.get(i).apply(motors, sensors);
        }

        return accumulator;
    }

    @Override
    public void runOpMode() {
        Motors motors = new Motors();
        Sensors sensors = new Sensors();
        motors.init(hardwareMap);
        sensors.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.speak("I have been initialized!");
        telemetry.update();
        waitForStart();
    }
}
