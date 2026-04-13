package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.apache.commons.math3.linear.RealMatrix;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;

@TeleOp
public class SimpleTeleOp extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private GamepadEx gamepad;

    // Localization
    private Pinpoint pinpoint;
    private Limelight limelight;
    private FusedLocalizer fusedLocalizer;

    private double targetRpm = 3000;
    private double targetTurretAngle = 0; // Center

    @Override
    public void onInit() {
        // 1. Hardware/Subsystems
        voltageSensor = new VoltageSensor();
        drivetrain = new MecanumDrivetrain(voltageSensor);
        intake = new Intake();
        turret = new Turret(voltageSensor);
        gamepad = new GamepadEx(gamepad1);

        // 2. Localization Stack
        // Initialize pinpoint at (0,0,0)
        pinpoint = new Pinpoint(new Transform(0, 0, 0));
        limelight = new Limelight();
        limelight.localizationPipeline(); // Set to Apriltag/Goal pipeline

        fusedLocalizer = new FusedLocalizer(
                pinpoint,
                limelight,
                new Transform(0, 0, 0)
        );

        // 3. Registering
        // Note: Registering localizer and pinpoint ensures their periodic() updates run
        register(voltageSensor, drivetrain, intake, turret, gamepad, pinpoint, limelight, fusedLocalizer);
    }

    @Override
    public void periodic() {
        // --- DRIVING ---
        double x = gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorX;
        double y = -gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorY;
        double theta = -gamepad.getRightX() / Constants.drivetrainPowerConversionFactorTheta;
        drivetrain.move(new Transform(x, y, theta));

        // --- INTAKE & TRANSFER ---
        if (gamepad.getRightTrigger() > 0.1) {
            intake.intake(true);
        } else if (gamepad.getLeftTrigger() > 0.1) {
            intake.intake(false);
        } else {
            intake.stop();
        }
        intake.transfer(gamepad.getA()); // Hold A to transfer

        // --- SHOOTER RPM (D-pad Up/Down) ---
        if (gamepad.getDpadUpJustPressed()) targetRpm += 100;
        if (gamepad.getDpadDownJustPressed()) targetRpm -= 100;

        if (gamepad.getRightBumper()) {
            turret.shoot(targetRpm);
        } else {
            turret.shoot(0);
        }

        // --- TURRET ANGLE (D-pad Left/Right) ---
        if (gamepad.getDpadRight()) targetTurretAngle += 0.005;
        if (gamepad.getDpadLeft()) targetTurretAngle -= 0.005;
        turret.turn(targetTurretAngle);

        // --- LOCALIZATION LOGGING ---
        Transform fusedPose = fusedLocalizer.getPosition();
        Transform rawPinpoint = pinpoint.getPosition();
        RealMatrix cov = fusedLocalizer.getCovariance();

        telemetry.addLine("--- Fused Localizer ---");
        telemetry.addData("Fused X", "%.2f", fusedPose.getX());
        telemetry.addData("Fused Y", "%.2f", fusedPose.getY());
        telemetry.addData("Fused Deg", "%.2f", Math.toDegrees(fusedPose.getTheta()));

        telemetry.addLine("--- Uncertainty (Covariance) ---");
        telemetry.addData("Var X", "%.4f", cov.getEntry(0, 0));
        telemetry.addData("Var Y", "%.4f", cov.getEntry(1, 1));
        telemetry.addData("Var Theta", "%.4f", cov.getEntry(2, 2));

        telemetry.addLine("--- Raw Pinpoint ---");
        telemetry.addData("PP X", "%.2f", rawPinpoint.getX());
        telemetry.addData("PP Y", "%.2f", rawPinpoint.getY());

        telemetry.addLine("--- Mechanism Stats ---");
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Turret Deg", Math.toDegrees(targetTurretAngle));

        telemetry.update();
    }
}