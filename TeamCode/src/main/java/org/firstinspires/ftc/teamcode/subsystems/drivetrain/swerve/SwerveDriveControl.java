package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;

public class SwerveDriveControl implements Command {
    private SwerveDrivetrain drivetrain;
    private Localizer localizer;

    private GamepadEx gamepad;

    private double targetHeading = 0;
    private boolean followTargetHeading = false;

    private PIDFAxis pidf;
    private Stopwatch stopwatch;

    public SwerveDriveControl(SwerveDrivetrain drivetrain, Localizer localizer, GamepadEx gamepad) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;

        this.gamepad = gamepad;

        pidf = new PIDFAxis(new PIDFAxis.K(Constants.pidPHeadingEnforcement, Constants.pidIHeadingEnforcement, Constants.pidDHeadingEnforcement, new double[]{}, 1, 1, Constants.pidTauHeadingEnforcement, Constants.pidGammaHeadingEnforcement));
        stopwatch = new Stopwatch();
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    @Override
    public void start() {
        targetHeading = localizer.getPosition().getTheta();
        pidf.reset();
        stopwatch.reset();
    }

    @Override
    public boolean periodic() {
        CommandRuntimeOpMode.getTelemetry().addData("Follow Target Heading", followTargetHeading);

        if (gamepad.getRightX() != 0) followTargetHeading = false;
        else if (!followTargetHeading && Math.abs(localizer.getVelocity().getTheta()) < Constants.swerveAngularVelocityCorrectionDeadzone) {
            followTargetHeading = true;
            targetHeading = localizer.getPosition().getTheta();

            pidf.reset();
        }

        if (followTargetHeading) {
            double heading = localizer.getPosition().getTheta();
            double control = -pidf.update(heading - targetHeading, new double[]{}, stopwatch.getDt());

            drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), control).toLocalVelocity(localizer.getPosition()));
        } else
            drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()).toLocalVelocity(localizer.getPosition()));

        return false;
    }
}