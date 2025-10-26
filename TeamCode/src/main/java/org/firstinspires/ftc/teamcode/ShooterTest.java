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

        // Example test parameters — you can modify these for experiments
        double distanceToTargetInches = 100.0; // horizontal distance to target
        double launchAngleDeg = 25.0;          // desired shooting angle (deg)
        double hoodContactLengthIn = 5.0;      // hood contact length (inches)

        // Compute required RPM for the given parameters
        Shooter.ShotParameters shot = Shooter.computeRequiredRpmForAngleInches(
                distanceToTargetInches,
                launchAngleDeg,
                hoodContactLengthIn
        );

        // Display results
        telemetry.addLine("=== Shooter Calculations ===");
        telemetry.addData("Distance (in)", "%.1f", distanceToTargetInches);
        telemetry.addData("Target Height (in)", "%.1f", Shooter.TARGET_HEIGHT_IN);
        telemetry.addData("Shooter Height (in)", "%.1f", Shooter.SHOOTER_EXIT_HEIGHT_IN);
        telemetry.addData("Launch Angle (deg)", "%.2f", shot.launchAngleDeg);
        telemetry.addData("Required RPM", "%.1f", shot.motorRpm);
        telemetry.addData("Feasible", shot.feasible ? "Yes" : "No");
        telemetry.addData("Message", shot.message);
        telemetry.update();

        // Keep showing telemetry until stopped
        while (opModeIsActive()) {
            idle();
        }
    }
}
