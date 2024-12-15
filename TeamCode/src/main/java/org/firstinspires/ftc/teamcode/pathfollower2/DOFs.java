package org.firstinspires.ftc.teamcode.pathfollower2;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;

public class DOFs {
    public double[] wheights = { 1.0, 1.0, 0.3, 1.0, 1.0, 1.0, 1.0 };
    public GoBildaPinpointDriver odometry;
    public Motors motors;

    public enum DOF {
        X,
        Y,
        THETA,
    }

    public double lastTime = 0;

    public HashMap<DOF, Double> getPosition() {
        HashMap<DOF, Double> positions = new HashMap<DOF, Double>();
        positions.put(DOF.X, odometry.getPosition().getX(DistanceUnit.INCH));
        positions.put(DOF.Y, odometry.getPosition().getY(DistanceUnit.INCH));
        positions.put(DOF.THETA, odometry.getPosition().getHeading(AngleUnit.DEGREES));
        return positions;
    }

    public void apply(HashMap<DOF, Double> gradient, Telemetry telemetry) {
        odometry.update();

        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double dx = wheights[0] * gradient.get(DOF.X);
        double dy = wheights[1] * gradient.get(DOF.Y);

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
                    leftFrontPower -= wheights[2] * delta;
                    rightFrontPower += wheights[2] * delta;
                    leftBackPower -= wheights[2] * delta;
                    rightBackPower += wheights[2] * delta;
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
        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightBackPower", rightBackPower);

        double n = 0.2;
        motors.leftFrontDrive.setPower(n * leftFrontPower);
        motors.rightFrontDrive.setPower(n * rightFrontPower);
        motors.leftBackDrive.setPower(n * leftBackPower);
        motors.rightBackDrive.setPower(n * rightBackPower);
    }

    public DOFs(GoBildaPinpointDriver odometry, Motors motors) {
        this.odometry = odometry;
        this.motors = motors;
    }
}
