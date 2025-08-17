package org.firstinspires.ftc.teamcode.pathfollower2;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;

public class DOFs {
    public GoBildaPinpointDriver odometry;
    public Motors motors;

    public enum DOF {
        X,
        Y,
        THETA,
    }

    public HashMap<DOF, Double> getPosition() {
        HashMap<DOF, Double> positions = new HashMap<>();
        positions.put(DOF.X, odometry.getPosition().getX(DistanceUnit.INCH));
        positions.put(DOF.Y, odometry.getPosition().getY(DistanceUnit.INCH));
        positions.put(DOF.THETA, odometry.getHeading() * 180 / Math.PI);

        return positions;
    }

    public void apply(HashMap<DOF, Double> gradient, Telemetry telemetry, double[] weights) {
        odometry.update();

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double dx = weights[0] * (Math.cos(-getPosition().get(DOF.THETA) * Math.PI / 180) * gradient.get(DOF.X) - Math.sin(-getPosition().get(DOF.THETA) * Math.PI / 180) * gradient.get(DOF.Y));
        double dy = weights[0] * (Math.sin(-getPosition().get(DOF.THETA) * Math.PI / 180) * gradient.get(DOF.X) + Math.cos(-getPosition().get(DOF.THETA) * Math.PI / 180) * gradient.get(DOF.Y));

        for (DOF dof : DOFs.DOF.values()) {
            telemetry.addData("dof", dof);
            double delta = gradient.get(dof);
            telemetry.addData("delta", delta);

            switch (dof) {
                case X:
                    leftFrontPower += dx;
                    rightFrontPower += dx;
                    leftBackPower += dx;
                    rightBackPower += dx;
                    break;
                case Y:
                    leftFrontPower -= dy;
                    rightFrontPower += dy;
                    leftBackPower += dy;
                    rightBackPower -= dy;
                    break;
                case THETA:
                    leftFrontPower -= weights[1] * delta;
                    rightFrontPower += weights[1] * delta;
                    leftBackPower -= weights[1] * delta;
                    rightBackPower += weights[1] * delta;
                    break;
            }
        }

        double max = Math.max(Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.abs(leftBackPower)), Math.abs(rightBackPower));

        if (max > 1) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontPower *= weights[2];
        rightFrontPower *= weights[3];
        leftBackPower *= weights[4];
        rightBackPower *= weights[5];

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightBackPower", rightBackPower);

        motors.leftFrontDrive.setPower(weights[6] * leftFrontPower);
        motors.rightFrontDrive.setPower(weights[6] * rightFrontPower);
        motors.leftBackDrive.setPower(weights[6] * leftBackPower);
        motors.rightBackDrive.setPower(weights[6] * rightBackPower);
    }

    public DOFs(GoBildaPinpointDriver odometry, Motors motors) {
        this.odometry = odometry;
        this.motors = motors;
    }
}
