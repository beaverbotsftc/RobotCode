package org.firstinspires.ftc.teamcode.limeLocator;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.collections.Motors;

import java.util.List;

public class LimeLocator {
    public Motors motors;
    public static double ObjPos(double ty) {
        double targetOffsetAngle_Vertical = ty;

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = -90.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 6;
        //h1

        // distance from the target to the floor
        double goalHeightInches = 0;
        //h2

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;

        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        // to tan = d
        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    private static double findShortestSideRotation(List<List<Double>> coords) {
        // Validate input
        if (coords.size() != 4 || coords.get(0).size() != 2) {
            throw new IllegalArgumentException("Input must be a list of 4 2D coordinates.");
        }

        // Calculate distances between consecutive points
        double side1 = distance(coords.get(0), coords.get(1));
        double side2 = distance(coords.get(1), coords.get(2));
        double side3 = distance(coords.get(2), coords.get(3));
        double side4 = distance(coords.get(3), coords.get(0));

        // Find the shortest side and its angle
        double minSide = Math.min(Math.min(side1, side2), Math.min(side3, side4));
        double angle = 0;

        if (minSide == side1) {
            angle = calculateAngle(coords.get(0), coords.get(1));
        } else if (minSide == side2) {
            angle = calculateAngle(coords.get(1), coords.get(2));
        } else if (minSide == side3) {
            angle = calculateAngle(coords.get(2), coords.get(3));
        } else if (minSide == side4) {
            angle = calculateAngle(coords.get(3), coords.get(0));
        }

        return angle;
    }

    // Helper function to calculate the distance between two points
    private static double distance(List<Double> point1, List<Double> point2) {
        return Math.sqrt(Math.pow(point2.get(0) - point1.get(0), 2) + Math.pow(point2.get(1) - point1.get(1), 2));
    }

    // Helper function to calculate the angle of a side relative to the x-axis
    private static double calculateAngle(List<Double> point1, List<Double> point2) {
        return Math.toDegrees(Math.atan2(point2.get(1) - point1.get(1), point2.get(0) - point1.get(0)));
    }

    public static double FindRotation(LLResult result) {
        double shortestSideRotation = 0;
        if (result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                List<List<Double>> corners = cr.getTargetCorners();
                shortestSideRotation = findShortestSideRotation(corners);

            }
        }

        return shortestSideRotation;
    }
    public static double ObjPos(double ty) {


        motors.leftFrontDrive.setPower(n * leftFrontPower);
        motors.rightFrontDrive.setPower(n * rightFrontPower);
        motors.leftBackDrive.setPower(n * leftBackPower);
        motors.rightBackDrive.setPower(n * rightBackPower);
    }
}