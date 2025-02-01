package org.firstinspires.ftc.teamcode.limeLocator;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.collections.Motors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LimeLocator {
    public Motors motors;

    private static final int MAX_HISTORY_SIZE = 50; // Max number of recent values to store
    private static List<Double> rotationHistory = new ArrayList<>();


    public static Map<String, List<Double>> assignCorners(List<List<Double>> coords) {
        // Sort by y-coordinate first (ascending), then by x-coordinate (ascending)
        coords.sort((a, b) -> {
            if (!a.get(1).equals(b.get(1))) {
                return Double.compare(a.get(1), b.get(1));
            } else {
                return Double.compare(a.get(0), b.get(0));
            }
        });

        // After sorting:
        // coords.get(0) -> Bottom-left
        // coords.get(1) -> Bottom-right
        // coords.get(2) -> Top-left
        // coords.get(3) -> Top-right

        Map<String, List<Double>> cornerMap = new HashMap<>();
        cornerMap.put("Bottom-left", coords.get(0));
        cornerMap.put("Bottom-right", coords.get(1));
        cornerMap.put("Top-left", coords.get(2));
        cornerMap.put("Top-right", coords.get(3));

        return cornerMap;
    }

    public static double findShortestSideRotation(List<List<Double>> coords) {
        double angle = 0;

        if (coords.size() >= 4) {
            Map<String, List<Double>> corners = assignCorners(coords);

            double side1 = distance(corners.get("Bottom-left"), corners.get("Bottom-right"));
            double side2 = distance(corners.get("Bottom-left"), corners.get("Top-left"));

            double minSide = Math.min(side1, side2);

            if (minSide == side1) {
                angle = calculateAngle(corners.get("Bottom-left"), corners.get("Bottom-right"));
            } else if (minSide == side2) {
                angle = calculateAngle(corners.get("Bottom-left"), corners.get("Top-left"));
            }
        }

        return angle;
    }

    private static double distance(List<Double> point1, List<Double> point2) {
        return Math.sqrt(Math.pow(point2.get(0) - point1.get(0), 2) + Math.pow(point2.get(1) - point1.get(1), 2));
    }

    private static double calculateAngle(List<Double> point1, List<Double> point2) {
        double angle = Math.toDegrees(Math.atan2(point2.get(1) - point1.get(1), point2.get(0) - point1.get(0)));
        // Normalize angle to the range [0, 360)
        angle = angle % 360;

        // If the angle is negative, adjust to positive range [0, 360)
        if (angle < 0) {
            angle += 360;
        }

        // Map the angle to the range [-180, 180]
        if (angle > 180) {
            angle -= 360;
        }




//


        return angle;
    }

    public static double FindRotation(LLResult result) {
        double shortestSideRotation = PullFromHistory();
        List<List<Double>> corners = null;
        rotationHistory = new ArrayList<>();
        ElapsedTime runtime = new ElapsedTime();
        double endTime = runtime.seconds() + 0.25;

        while(runtime.seconds() < endTime) {

        if (result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                corners = cr.getTargetCorners();
                shortestSideRotation = findShortestSideRotation(corners);
            }

            // Add the rotation to the history and calculate the average
            if (shortestSideRotation != 0) {
                addToHistory(shortestSideRotation);
            }


         }
        }


        return getAverageRotation();
    }

    private static void addToHistory(double rotation) {
        if (rotationHistory.size() >= MAX_HISTORY_SIZE) {
            rotationHistory.remove(0); // Remove the oldest value
        }
        rotationHistory.add(rotation);
    }
    private static double PullFromHistory() {
        if (rotationHistory.size() == 0){
            return 0;
        }else{
            return rotationHistory.get(rotationHistory.size() -1);
    }
    }
    public static List PullHistory() {

        return rotationHistory;
    }

    private static double getAverageRotation() {
        if (rotationHistory.isEmpty()) return 0;

        double sum = 0;
        for (double rotation : rotationHistory) {
            sum += rotation;
        }
        return sum / rotationHistory.size();
    }

    public void Position(double tncy, double tncx) {
        int padding = 3;

        while ((Math.abs(tncy) >= padding) && (Math.abs(tncx) >= padding)) {
            double max;
            double speed = 0.25;
            double x = tncx / 26;
            double y = tncy / 26;
            double yP = -y;
            double xP = -x;
            double axial = yP * -speed;
            double lateral = xP * speed;

            double leftFrontPower = (axial + lateral);
            double rightFrontPower = (axial - lateral);
            double leftBackPower = (axial - lateral);
            double rightBackPower = (axial + lateral);

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

          //  motors.leftFrontDrive.setPower(leftFrontPower);
           // motors.rightFrontDrive.setPower(rightFrontPower);
           // motors.leftBackDrive.setPower(leftBackPower);
           // motors.rightBackDrive.setPower(rightBackPower);
        }
    }
}
