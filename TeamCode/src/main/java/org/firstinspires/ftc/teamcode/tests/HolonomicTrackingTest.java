package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.PIDFAxis;
import org.beaverbots.beaver.pathing.Path;
import org.beaverbots.beaver.pathing.PathAxis;
import org.firstinspires.ftc.teamcode.tests.utils.FakeLocomotionAndLocalization;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "Tests")
public class HolonomicTrackingTest extends CommandRuntimeOpMode {
    FakeLocomotionAndLocalization locomotionAndLocalization = new FakeLocomotionAndLocalization(FakeLocomotionAndLocalization.LocomotionType.HOLONOMIC, 1, 1, 0.5, 0.5, 0, 0, 0, "Fake locomotion and localization");

    Command tracker = new HolonomicFollowPath(
            new Path(
                    List.of(
                            new PathAxis(t -> Math.cos(t), 0, 10),
                            new PathAxis(t -> Math.sin(t), 0, 10),
                            new PathAxis(t -> t, 0, 10)
                    ),
                    t -> t >= 10),

            new PIDF(List.of(
                    new PIDFAxis(new PIDFAxis.K(1, 0.01, 0.1, 1, 1, 1, 0.05, 0)),
                    new PIDFAxis(new PIDFAxis.K(1, 0.01, 0.1, 1, 1, 1, 0.05, 0)),
                    new PIDFAxis(new PIDFAxis.K(1, 0.01, 0.1, 1, 1, 1, 0.05, 0))
            )),
            locomotionAndLocalization,
            locomotionAndLocalization
    );

    List<Double> time = new ArrayList<>();
    List<Double> x = new ArrayList<>();
    List<Double> y = new ArrayList<>();
    List<Double> theta = new ArrayList<>();

    ElapsedTime clock = null;

    @Override
    protected void onStart() {
        schedule(tracker);
        clock = new ElapsedTime();
    }

    boolean done = false;
    @Override
    protected void periodic() {
        if (done) return;
        time.add(clock.time());
        x.add(locomotionAndLocalization.getPositionAsList().get(0));
        y.add(locomotionAndLocalization.getPositionAsList().get(1));
        theta.add(locomotionAndLocalization.getPositionAsList().get(2));

        if (!getRunningCommands().contains(tracker) && clock.seconds() > 1 && !done) {
            String output = "import matplotlib.pyplot as plt\n\n";

            output += "x = [\n";
            for (int i = 0; i < time.size(); i++) {
                output += "(" + time.get(i) + "," + x.get(i) + "),\n";
            }
            output += "]\n\n";

            output += "y = [\n";
            for (int i = 0; i < time.size(); i++) {
                output += "(" + time.get(i) + "," + y.get(i) + "),\n";
            }
            output += "]\n\n";

            output += "theta = [\n";
            for (int i = 0; i < time.size(); i++) {
                output += "(" + time.get(i) + "," + theta.get(i) + "),\n";
            }
            output += "]\n\n";

            output += "\n\n";
            output += "time_x, x_vals = zip(*x)\n";
            output += "time_y, y_vals = zip(*y)\n";
            output += "time_theta, theta_vals = zip(*theta)\n\n";

            output += "plt.figure(figsize=(12, 6))\n";
            output += "plt.plot(time_x, x_vals, label='x')\n";
            output += "plt.plot(time_y, y_vals, label='y')\n";
            output += "plt.plot(time_theta, theta_vals, label='theta')\n\n";

            output += "plt.title('x, y, and theta vs Time')\n";
            output += "plt.xlabel('Time')\n";
            output += "plt.ylabel('Values')\n";
            output += "plt.legend()\n";
            output += "plt.grid(True)\n";
            output += "plt.show()\n";

            logLongString(output);
            done=true;
        }
    }

    void logLongString(String str) {
        StringBuilder output = new StringBuilder();
        for (String x : str.split("\n")) {
            if (output.length() + x.length() > 4000) {
                RobotLog.i(output.toString());
                output = new StringBuilder();
            }
            output.append(x + "\n");
        }
        RobotLog.i(output.toString());
    }
}
