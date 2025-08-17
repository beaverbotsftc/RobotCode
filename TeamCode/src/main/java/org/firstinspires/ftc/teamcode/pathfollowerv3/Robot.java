package org.firstinspires.ftc.teamcode.pathfollowerv3;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pinpoint.GoBildaPinpointDriver;

import java.util.HashMap;
import java.util.function.Supplier;

public class Robot {
    public static Robot robot;

    public enum DOF {
        X,
        Y,
        Theta,
    }

    private final DcMotor leftFrontDrivetrainMotor;
    private final DcMotor leftBackDrivetrainMotor;
    private final DcMotor rightFrontDrivetrainMotor;
    private final DcMotor rightBackDrivetrainMotor;

    private final GoBildaPinpointDriver odometry;

    private HashMap<DOF, Double> position;

    private PathFollower pathFollower;
    private boolean pathSegmentCompleted;

    private double lastTime;
    private double deltaTime;

    public final Supplier<Boolean> isStopRequested;

    public final Telemetry telemetry;

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

        robot = this;
    }

    private HashMap<DOF, Double> getNewPosition() {
        HashMap<DOF, Double> positions = new HashMap<>();
        positions.put(DOF.X, odometry.getPosition().getX(DistanceUnit.INCH));
        positions.put(DOF.Y, odometry.getPosition().getY(DistanceUnit.INCH));
        positions.put(DOF.Theta, odometry.getHeading() * 180 / Math.PI);

        return positions;
    }

    public HashMap<DOF, Double> getPosition() {
        return position;
    }

    public void resetPosition() {
        odometry.resetPosAndIMU();
        try { Thread.sleep(1000); } catch (InterruptedException e) { Thread.currentThread().interrupt(); } // Wait for odometry reset completion
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    // Must be called before anything else
    public void tick() {
        odometry.update();

        position = getNewPosition();

        double currentTime = (double) System.currentTimeMillis() * 1e-3;
        deltaTime = currentTime - lastTime;
        if (deltaTime < 0.001) return;
        lastTime = currentTime;

        if (pathFollower != null) {
            Utils.Ternary<HashMap<DOF, Double>, PathFollower.Status, Boolean> movement = pathFollower.tick(deltaTime, getPosition());

            move(movement.x1);

            if (movement.x2 == PathFollower.Status.Complete) {
                pathFollower = null;
            }

            pathSegmentCompleted = movement.x3;
        }
    }

    public boolean isFollowingPath() {
        return pathFollower != null;
    }

    public boolean isPathSegmentCompleted() {
        return pathSegmentCompleted;
    }

    public void followPath(Path path, DOF[] dofs) {
        pathFollower = new PathFollower(dofs, path,
                new HashMap<DOF, PID.K>() {{
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
                }},
                new HashMap<DOF, PathFollower.K>() {{
                    put(DOF.X, new PathFollower.K(
                        Constants.weights.get(Constants.Constant.PathFollowerXDeviationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerXVelocityGain),
                        Constants.weights.get(Constants.Constant.PathFollowerXAccelerationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerXConstant)
                    ));
                    put(DOF.Y, new PathFollower.K(
                        Constants.weights.get(Constants.Constant.PathFollowerYDeviationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerYVelocityGain),
                        Constants.weights.get(Constants.Constant.PathFollowerYAccelerationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerYConstant)
                    ));
                    put(DOF.Theta, new PathFollower.K(
                        Constants.weights.get(Constants.Constant.PathFollowerThetaDeviationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerThetaVelocityGain),
                        Constants.weights.get(Constants.Constant.PathFollowerThetaAccelerationGain),
                        Constants.weights.get(Constants.Constant.PathFollowerThetaConstant)
                    ));
                }},
                getPosition());
    }

    public void move(HashMap<DOF, Double> movement) {
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double dx = Constants.weights.get(Constants.Constant.LateralSpeed)
                    * (Math.cos(-getPosition().get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.X) - Math.sin(-getPosition().get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.Y));
        double dy = Constants.weights.get(Constants.Constant.LateralSpeed)
                    * (Math.sin(-getPosition().get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.X) + Math.cos(-getPosition().get(DOF.Theta) * Math.PI / 180) * movement.get(DOF.Y));

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