package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Shooter Test Initialized");
        telemetry.update();

        waitForStart();

        // Example distance to target (in inches)
        double distanceToTargetInches = 100.0; // change this for testing

        // Compute shot parameters from Shooter utility
        Shooter.ShotParameters shot = Shooter.computeSlightArcForDistanceInches(distanceToTargetInches);

        if (shot.feasible) {
            telemetry.addLine("=== Shooter Calculations ===");
            telemetry.addData("Distance (in)", distanceToTargetInches);
            telemetry.addData("Target Height (in)", Shooter.TARGET_HEIGHT_IN);
            telemetry.addData("Shooter Height (in)", Shooter.SHOOTER_EXIT_HEIGHT_IN);
            telemetry.addData("Angle (deg)", "%.2f", shot.angleDeg);
            telemetry.addData("Required RPM", "%.2f", shot.motorRpm);
            telemetry.addData("Message", shot.message);
        } else {
            telemetry.addLine("Shot not feasible!");
            telemetry.addData("Reason", shot.message);
        }

        telemetry.update();

        // Keep showing telemetry until stop
        while (opModeIsActive()) {
            idle();
        }
    }
}
