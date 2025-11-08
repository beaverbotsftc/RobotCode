package org.firstinspires.ftc.teamcode.experiments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(group = "Experiments")
public class AprilTagTracking extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor limelightMotor;

    // Location of Tag 20 in field (meters)
    private static final double targetX = -1.481376;
    private static final double targetY = -1.388042;

    // Motor setup (adjust TICKS_PER_REV to your motor spec)
    private static final int TICKS_PER_REV = 1440; // typical motor encoder ticks per revolution
    private static final double GEAR_RATIO = 1.5;  // motor:turret ratio
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV / 360.0) / GEAR_RATIO;

    // Store last known target for when vision is lost
    private Integer lastTargetTicks = null;
    private Double lastTargetAngle = null;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightMotor = hardwareMap.get(DcMotor.class, "Motor");

        // Reset encoder
        limelightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        limelightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start Limelight
        limelight.pipelineSwitch(9); // AprilTag field map pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result.isValid() && result.getBotpose() != null) {
                Pose3D botPose = result.getBotpose();

                double robotX = botPose.getPosition().x;
                double robotY = botPose.getPosition().y;
                double robotYaw = botPose.getOrientation().getYaw(); // degrees

                // Compute angle to target
                double angleToTarget = calculateTargetAngle(robotX, robotY);

                // Compute shortest rotation error
                double error = getShortestAngleError(angleToTarget, robotYaw);

                // Convert error into encoder ticks
                int targetTicks = (int)(error * TICKS_PER_DEGREE);

                // Flip direction to match motor orientation
                targetTicks = -targetTicks;

                // Calculate new absolute motor target
                int newTarget = limelightMotor.getCurrentPosition() + targetTicks;

                // Move motor with moderate power for smooth tracking
                limelightMotor.setTargetPosition(newTarget);
                limelightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                limelightMotor.setPower(0.7); // adjust 0.2-0.4 for smoothness

                // Save last known target
                lastTargetTicks = newTarget;
                lastTargetAngle = angleToTarget;

                telemetry.addData("Tracking", "ACTIVE");
                telemetry.addData("Robot Pose", "(%.2f, %.2f) Yaw: %.2f", robotX, robotY, robotYaw);
                telemetry.addData("Target", "(%.2f, %.2f)", targetX, targetY);
                telemetry.addData("AngleToTarget", angleToTarget);
                telemetry.addData("Error", error);
                telemetry.addData("Motor Target", newTarget);

            } else {
                // No valid pose → rotate toward last known target using shortest path
                if (lastTargetAngle != null) {
                    // Get current yaw from latest pose if available
                    limelightMotor.setPower(0);

                } else {
                    // If never saw tag, stop motor
                    limelightMotor.setPower(0);
                    telemetry.addData("Tracking", "NO DATA");
                }
            }



            telemetry.update();
        }

        limelight.stop();
    }

    // Separate function to calculate the target angle for the turret
    private double calculateTargetAngle(double robotX, double robotY) {
        // Default: aim directly at Tag 20
        double dx = targetX - robotX;
        double dy = targetY - robotY;
        return Math.toDegrees(Math.atan2(dy, dx)) * 1;
    }

    // Utility: shortest path error between target and current angle
    private double getShortestAngleError(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
