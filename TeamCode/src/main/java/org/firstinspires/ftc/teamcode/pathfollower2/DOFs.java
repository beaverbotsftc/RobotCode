package org.firstinspires.ftc.teamcode.pathfollower2;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.collections.Motors;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;

public class DOFs {
    private Motors motors;
    private GoBildaPinpointDriver odometry;
    public double[] wheights = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    public enum DOF {
        X,
        Y,
        THETA,
    }

    public HashMap<DOF, Double> getPositions() {
        HashMap<DOF, Double> positions = new HashMap<DOF, Double>();
        positions.put(DOF.X, odometry.getPosition().getX(DistanceUnit.INCH));
        positions.put(DOF.Y, odometry.getPosition().getY(DistanceUnit.INCH));
        positions.put(DOF.THETA, odometry.getPosition().getHeading(AngleUnit.DEGREES));
        return positions;
    }

    public void apply(HashMap<DOF, Double> gradient) {
        double leftFrontPower  = 0;
        double rightFrontPower = 0;
        double leftBackPower   = 0;
        double rightBackPower  = 0;

        HashMap<DOF, Double> positions = getPositions();

        for (DOF dof : gradient.keySet()) {
            double delta = gradient.get(dof);
            switch (dof) {
                case X:
                    leftFrontPower += wheights[0] * (gradient.get(DOF.X) * Math.cos(-positions.get(DOF.THETA)) - gradient.get(DOF.Y) * Math.sin(-gradient.get(DOF.THETA)));
                    rightFrontPower += wheights[0] * (gradient.get(DOF.X) * Math.cos(-positions.get(DOF.THETA)) - gradient.get(DOF.Y) * Math.sin(-gradient.get(DOF.THETA)));
                    leftBackPower += wheights[0] * (gradient.get(DOF.X) * Math.cos(-positions.get(DOF.THETA)) - gradient.get(DOF.Y) * Math.sin(-gradient.get(DOF.THETA)));
                    rightBackPower += wheights[0] * (gradient.get(DOF.X) * Math.cos(-positions.get(DOF.THETA)) - gradient.get(DOF.Y) * Math.sin(-gradient.get(DOF.THETA)));
                case Y:
                    leftFrontPower -= wheights[0] * (gradient.get(DOF.X) * Math.sin(-positions.get(DOF.THETA)) + gradient.get(DOF.Y) * Math.cos(-gradient.get(DOF.THETA)));
                    rightFrontPower += wheights[0] * (gradient.get(DOF.X) * Math.sin(-positions.get(DOF.THETA)) + gradient.get(DOF.Y) * Math.cos(-gradient.get(DOF.THETA)));
                    leftBackPower += wheights[0] * (gradient.get(DOF.X) * Math.sin(-positions.get(DOF.THETA)) + gradient.get(DOF.Y) * Math.cos(-gradient.get(DOF.THETA)));
                    rightBackPower -= wheights[0] * (gradient.get(DOF.X) * Math.sin(-positions.get(DOF.THETA)) + gradient.get(DOF.Y) * Math.cos(-gradient.get(DOF.THETA)));
                case THETA:
                    leftFrontPower += wheights[2] * delta;
                    rightFrontPower -= wheights[2] * delta;
                    leftBackPower += wheights[2] * delta;
                    rightBackPower -= wheights[2] * delta;
            }
        }

        leftFrontPower *= wheights[3];
        rightFrontPower *= wheights[4];
        leftBackPower *= wheights[5];
        rightBackPower *= wheights[6];

        motors.leftFrontDrive.setPower(leftFrontPower);
        motors.rightFrontDrive.setPower(rightFrontPower);
        motors.leftBackDrive.setPower(leftBackPower);
        motors.rightBackDrive.setPower(rightBackPower);
    }

    public DOFs(Motors motors, GoBildaPinpointDriver odometry) {
        this.motors = motors;
        this.odometry = odometry;
    }
}
