package org.firstinspires.ftc.teamcode.experiments;

import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.localizer.FusedLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.commands.SimpleControl;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;

@Autonomous
public class FusedLocalizationExperiment extends CommandRuntimeOpMode {
    private VoltageSensor voltageSensor;
    private Gamepad gamepad;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Pinpoint pinpoint;
    private FusedLocalizer fusedLocalizer;
    private Limelight limelight;

    @Override
    public void onInit() {
        voltageSensor = new VoltageSensor();
        gamepad = new Gamepad(gamepad1);
        drivetrain = new MecanumDrivetrain();
        intake = new Intake();
        shooter = new Shooter(voltageSensor);
        pinpoint = new Pinpoint(new DrivetrainState(8, 8, 0));
        limelight = new Limelight();
        fusedLocalizer = new FusedLocalizer(pinpoint, limelight, new DrivetrainState(8, 8, 0));
    }

    @Override
    public void onStart() {
        register(voltageSensor, gamepad, drivetrain, intake, shooter, pinpoint, limelight, fusedLocalizer);
        schedule(new SimpleControl(gamepad, drivetrain, intake, shooter));
    }

    private DrivetrainState lastGood;

    @Override
    public void periodic() {
        telemetry.addLine("Pinpoint pos: " + pinpoint.getPosition().toString());
        DrivetrainState now = limelight.getEstimatedPosition();
        if (now != null) lastGood = now;
        telemetry.addLine("Limelight pos: " + lastGood);
        telemetry.addLine("Fused pos: " + fusedLocalizer.getPosition().toString());
        telemetry.addLine("Vel: " + pinpoint.getVelocity().toString());
        if (gamepad.getDpadUpJustPressed())
            pinpoint.setPosition(fusedLocalizer.getPosition());
    }
}
