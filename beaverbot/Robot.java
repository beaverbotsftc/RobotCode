package org.firstinspires.ftc.teamcode.pathfollowerv3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;
import java.util.function.Supplier;

public class Robot {
    public enum DOF {
        X,
        Y,
        Theta,
    }

    public DcMotor leftFrontDrivetrainMotor;
    public DcMotor leftBackDrivetrainMotor;
    public DcMotor rightFrontDrivetrainMotor;
    public DcMotor rightBackDrivetrainMotor;

    public GoBildaPinpointDriver odometry;

    public Supplier<Boolean> isStopRequested;

    public HashMap<DOF, Double> position;

    public PathFollower pathFollower;

    double lastTime;

    Telemetry telemetry;

    public Robot(HardwareMap hardwareMap, Supplier<Boolean> isStopRequested, Telemetry telemetry) {
        this.telemetry = telemetry;

        this.isStopRequested = isStopRequested;

        // Drivetrain
        leftFrontDrivetrainMotor = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrivetrainMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrivetrainMotor = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrivetrainMotor = hardwareMap.get(DcMotor.class, "right_back");

        leftFrontDrivetrainMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrivetrainMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrivetrainMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrivetrainMotor.setDirection(DcMotor.Direction.FORWARD);

        // Pinpoint
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odometry.setOffsets(
                Constants.weights.get(Constants.Constant.OdometryOffsetX),
                Constants.weights.get(Constants.Constant.OdometryOffsetY)
        );

        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odometry.resetPosAndIMU();
        odometry.update();

        lastTime = (double) System.currentTimeMillis() * 1e-3;
    }


    private HashMap<DOF, Double> getPosition() {
        HashMap<DOF, Double> positions = new HashMap<>();
        positions.put(DOF.X, odometry.getPosition().getX(DistanceUnit.INCH));
        positions.put(DOF.Y, odometry.getPosition().getY(DistanceUnit.INCH));
        positions.put(DOF.Theta, odometry.getHeading() * 180 / Math.PI);

        return positions;
    }

    public void tick() {
        double currentTime = (double) System.currentTimeMillis() * 1e-3;
        double dt = currentTime - lastTime;
        if (dt < 0.001) return;
        lastTime = currentTime;

        odometry.update();

        position = getPosition();

        if (pathFollower != null) {
            HashMap<DOF, Double> movement = pathFollower.tick(dt, position);
            if (movement != null) {
                move(movement);
            } else {
                pathFollower = null;
            }
        }
    }

    public void followPath(Path path) {
        pathFollower = new PathFollower(this, new DOF[]{ DOF.X, DOF.Y, DOF.Theta }, path, new HashMap<DOF, PID.K>() {{
            put(DOF.X, new PID.K(
                Constants.weights.get(Constants.Constant.PidXProportional),
                Constants.weights.get(Constants.Constant.PidXIntegral),
                Constants.weights.get(Constants.Constant.PidXDerivative)
            ));

            put(DOF.Y, new PID.K(
                Constants.weights.get(Constants.Constant.PidYProportional),
                Constants.weights.get(Constants.Constant.PidYIntegral),
                Constants.weights.get(Constants.Constant.PidYDerivative)
            ));

            put(DOF.Theta, new PID.K(
                Constants.weights.get(Constants.Constant.PidThetaProportional),
                Constants.weights.get(Constants.Constant.PidThetaIntegral),
                Constants.weights.get(Constants.Constant.PidThetaDerivative)
            ));
        }}, new HashMap<DOF, PathFollower.K>() {{
            TODO: NEED TO ADD CONSTANTS TO THIS!!! NOT IN A COMMENT BECAUSE THIS IS CRITICAL
            put(DOF.X, new PathFollower.K(0.0, 0.0, 0.0));
            put(DOF.Y, new PathFollower.K(0.0, 0.0, 0.0));
            put(DOF.Theta, new PathFollower.K(0.0, 0.0, 0.0));
        }}, position, isStopRequested);
    }

    public void move(HashMap<DOF, Double> movement) {
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double dx = Constants.weights.get(Constants.Constant.LateralSpeed)
                    * (Math.cos(-position.get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.X) - Math.sin(-position.get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.Y));
        double dy = Constants.weights.get(Constants.Constant.LateralSpeed)
                    * (Math.sin(-position.get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.X) + Math.cos(-position.get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.Y));

        // X
        leftFrontPower += dx;
        rightFrontPower += dx;
        leftBackPower += dx;
        rightBackPower += dx;

        // Y
        leftFrontPower -= dy;
        rightFrontPower += dy;
        leftBackPower += dy;
        rightBackPower -= dy;

        // THETA
        leftFrontPower -= Constants.weights.get(Constants.Constant.RotationalSpeed) * movement.get(DOF.Theta);
        rightFrontPower += Constants.weights.get(Constants.Constant.RotationalSpeed) * movement.get(DOF.Theta);
        leftBackPower -= Constants.weights.get(Constants.Constant.RotationalSpeed) * movement.get(DOF.Theta);
        rightBackPower += Constants.weights.get(Constants.Constant.RotationalSpeed) * movement.get(DOF.Theta);

        double maxMotorPower = Math.max(Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.abs(leftBackPower)), Math.abs(rightBackPower));
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftBackPower /= maxMotorPower;
            rightBackPower /= maxMotorPower;
        }

        leftFrontPower *= Constants.weights.get(Constants.Constant.LeftFrontMotorPower);
        rightFrontPower *= Constants.weights.get(Constants.Constant.RightFrontMotorPower);
        leftBackPower *= Constants.weights.get(Constants.Constant.LeftBackMotorPower);
        rightBackPower *= Constants.weights.get(Constants.Constant.RightBackMotorPower);

        leftFrontDrivetrainMotor.setPower(Constants.weights.get(Constants.Constant.MaxDrivetrainMotorPower) * leftFrontPower);
        rightFrontDrivetrainMotor.setPower(Constants.weights.get(Constants.Constant.MaxDrivetrainMotorPower) * rightFrontPower);
        leftBackDrivetrainMotor.setPower(Constants.weights.get(Constants.Constant.MaxDrivetrainMotorPower) * leftBackPower);
        rightBackDrivetrainMotor.setPower(Constants.weights.get(Constants.Constant.MaxDrivetrainMotorPower) * rightBackPower);
    }
}