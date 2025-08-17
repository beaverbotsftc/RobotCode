package org.firstinspires.ftc.teamcode.pathfollowerv3;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "TuningOmnibus")
public class TuningOmnibus extends LinearOpMode {
    double time = 3;

    double testPathFollowerSingleDOF(double endPoint, Robot.DOF dof) {
        Robot.robot.followPath(
                new PathBuilder(new Robot.DOF[] { dof })
                        .startingPoint(dof, 0.0)
                        .linearTo(dof, endPoint, time)
                        .buildSegment()
                        .build(),
                new Robot.DOF[] { dof });

        while (!isStopRequested() && Robot.robot.isFollowingPath()) {
            Robot.robot.tick();
        }

        return Math.pow(Robot.robot.getPosition().get(dof) - endPoint, 2);
    }

    /// Returns false if skipped
    public boolean waitForNextTest(String testName, String description) {
        telemetry.addData("Next test", testName);
        telemetry.addLine("Waiting for confirmation, press O");
        telemetry.addLine("To skip test, press X");
        telemetry.addLine("If skipping this test, note that stability will drop if you do not skip the same tests every time");
        telemetry.addLine(description);

        final boolean output;

        while (!isStopRequested() && (!gamepad1.circle && !gamepad1.cross)) {}
        output = gamepad1.circle; // If the circle was pressed, do the test. Otherwise, pass. Also, if stop is requested, it will auto-skip because circle will not be pressed.

        while (!isStopRequested() && (gamepad1.circle || gamepad1.cross)) {}

        Robot.robot.resetPosition();

        return output;
    }

    public void runOpMode() {
        new Robot(hardwareMap, this::isStopRequested, telemetry);

        double baselineLoss = Double.POSITIVE_INFINITY;
        double newLoss = Double.NaN;

        int weightSelected = 0;

        double alpha = 0.01;

        waitForStart();

        double lastTime = (double) System.currentTimeMillis() * 1e-3;

        boolean lastLeft = false;
        boolean lastRight = false;

        while (opModeIsActive()) {
            final HashMap<Constants.Constant, Double> baselineWeights = new HashMap<>(Constants.weights);
            while (!gamepad1.circle && baselineLoss != Double.POSITIVE_INFINITY) {
                double deltaTime = (double) System.currentTimeMillis() * 1e-3 - lastTime;
                if (deltaTime <= 0.001)
                    continue;
                lastTime = (double) System.currentTimeMillis() * 1e-3;

                telemetry.addData("Maximum weight index", Constants.Constant.values().length);
                telemetry.addData("Current selected weight index", weightSelected);
                telemetry.addData("Current selected weight", Constants.Constant.values()[weightSelected]);

                telemetry.addData("Time", time);

                telemetry.addData("Alpha", alpha);

                telemetry.addLine("----------");
                telemetry.addLine("O to continue");
                telemetry.addLine("L and R to change alpha");
                telemetry.addLine("D-pad left & right to change selected weight");
                telemetry.addLine("D-pad up & down to change the selected weight's value");
                telemetry.addLine("Remember that X is forward!!!");
                telemetry.addLine("ZL and ZR to change time");

                telemetry.addLine("----------");
                telemetry.addData("Baseline weights", baselineWeights);
                telemetry.addData("Baseline loss", baselineLoss);
                telemetry.addData("New weights", Constants.weights);
                telemetry.addData("New loss", newLoss);

                if (gamepad1.left_bumper)
                    alpha -= deltaTime * 0.1;

                if (gamepad1.right_bumper)
                    alpha += deltaTime * 0.1;

                if (gamepad1.dpad_right && !lastRight)
                    weightSelected = (weightSelected + 1) % Constants.Constant.values().length;

                if (gamepad1.dpad_left && !lastLeft)
                    weightSelected = (weightSelected - 1 + Constants.Constant.values().length) % Constants.Constant.values().length;

                if (gamepad1.dpad_up)
                    Constants.weights.put(Constants.Constant.values()[weightSelected],
                            Constants.weights.get(Constants.Constant.values()[weightSelected]) + deltaTime * 0.5);

                if (gamepad1.dpad_down)
                    Constants.weights.put(Constants.Constant.values()[weightSelected],
                            Constants.weights.get(Constants.Constant.values()[weightSelected]) - deltaTime * 0.5);

                time -= gamepad1.left_trigger * deltaTime * 1;
                time += gamepad1.right_trigger * deltaTime * 1;

                telemetry.update();

                lastRight = gamepad1.dpad_right;
                lastLeft = gamepad1.dpad_left;
            }

            HashMap<Constants.Constant, Double> deltaWeights = Constants.weights.entrySet().stream().collect(HashMap::new,
                    (HashMap<Constants.Constant, Double> map, Map.Entry<Constants.Constant, Double> entry)
                            -> map.put(entry.getKey(), entry.getValue() - baselineWeights.get(entry.getKey())),
                    HashMap::putAll);

            newLoss = 0;

            if (waitForNextTest("Path follower test move forward", "Moves forward 12 inches"))
                newLoss += testPathFollowerSingleDOF(12.0, Robot.DOF.X);

            if (waitForNextTest("Path follower test move forward", "Moves forward 24 inches"))
                newLoss += testPathFollowerSingleDOF(24.0, Robot.DOF.X);

            if (waitForNextTest("Path follower test move forward", "Moves forward 36 inches"))
                newLoss += testPathFollowerSingleDOF(36.0, Robot.DOF.X);

            if (waitForNextTest("Path follower test move backward", "Moves backward 12 inches"))
                newLoss += testPathFollowerSingleDOF(-12.0, Robot.DOF.X);

            if (waitForNextTest("Path follower test move right", "Moves right 12 inches"))
                newLoss += testPathFollowerSingleDOF(12.0, Robot.DOF.Y);

            if (waitForNextTest("Path follower test move right", "Moves right 24 inches"))
                newLoss += testPathFollowerSingleDOF(24.0, Robot.DOF.Y);

            if (waitForNextTest("Path follower test move right", "Moves right 36 inches"))
                newLoss += testPathFollowerSingleDOF(36.0, Robot.DOF.Y);

            if (waitForNextTest("Path follower test move left", "Moves left 12 inches"))
                newLoss += testPathFollowerSingleDOF(-12.0, Robot.DOF.Y);

            if (waitForNextTest("Path follower test turn clockwise", "Turns clockwise 90 degrees"))
                newLoss += testPathFollowerSingleDOF(90.0, Robot.DOF.Theta);

            if (waitForNextTest("Path follower test turn clockwise", "Turns clockwise 180 degrees"))
                newLoss += testPathFollowerSingleDOF(180.0, Robot.DOF.Theta);

            if (waitForNextTest("Path follower test turn clockwise", "Turns clockwise 720 degrees"))
                newLoss += testPathFollowerSingleDOF(720.0, Robot.DOF.Theta);

            if (waitForNextTest("Path follower test turn counterclockwise", "Turns counterclockwise 720 degrees"))
                newLoss += testPathFollowerSingleDOF(-720.0, Robot.DOF.Theta);

            if (baselineLoss == Double.POSITIVE_INFINITY) {
                baselineLoss = newLoss;
                continue;
            }

            double deltaLoss = newLoss - baselineLoss;

            while (!gamepad1.circle) {
                telemetry.addLine("Press O to continue");
                telemetry.addData("Old weights", baselineWeights);
                telemetry.addData("Delta loss", deltaLoss);
            }

            final double capturedAlpha = alpha;
            Constants.weights = Constants.weights.entrySet().stream().collect(HashMap::new,
                    (HashMap<Constants.Constant, Double> map, Map.Entry<Constants.Constant, Double> entry)
                            -> map.put(entry.getKey(),
                            baselineWeights.get(entry.getKey())
                            - capturedAlpha * deltaLoss * deltaWeights.get(entry.getKey())),
                    HashMap::putAll);
        }
    }
}