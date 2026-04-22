package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Stopwatch;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;
import java.util.function.DoubleUnaryOperator;

public class SwerveDriveControl implements Command {
    private SwerveDrivetrain drivetrain;
    private Localizer localizer;

    private GamepadEx gamepad;
    private DoubleUnaryOperator[] mirror;

    private double targetHeading = 0;
    private boolean followTargetHeading = false;

    private PIDFAxis pidf;
    private Stopwatch stopwatch;

    ///  Note that this isn't the standard field mirroring, the symmetries are different.
    /// x -> -x, y -> -y, theta -> theta
    public SwerveDriveControl(SwerveDrivetrain drivetrain, Localizer localizer, GamepadEx gamepad, DoubleUnaryOperator[] mirror) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;

        this.gamepad = gamepad;
        this.mirror = mirror;

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
        CommandOpMode.addData("Follow Target Heading", followTargetHeading);

        if (gamepad.getRightX() != 0 || new Transform(gamepad.getLeftX(), gamepad.getLeftY()).lateralNorm() < Constants.headingEnforcementLateralForceCutoff/* || localizer.getVelocity().lateralNorm() < Constants.headingEnforcementLateralSpeedCutoff*/)
            followTargetHeading = false;
        else if (!followTargetHeading && Math.abs(localizer.getVelocity().getTheta()) < Constants.swerveAngularVelocityCorrectionDeadzone) {
            followTargetHeading = true;
            targetHeading = localizer.getPosition().getTheta();

            pidf.reset();
        }

        if (followTargetHeading) {
            double heading = localizer.getPosition().getTheta();
            double control = -pidf.update(heading - targetHeading, new double[]{}, stopwatch.getDt());

            drivetrain.move(new Transform(gamepad.getLeftX(), gamepad.getLeftY(), control).transform(mirror).toLocalVelocity(localizer.getPosition()));
        } else
            drivetrain.move(new Transform(gamepad.getLeftX(), gamepad.getLeftY(), -Math.pow(gamepad.getRightX(), 3)).transform(mirror).toLocalVelocity(localizer.getPosition()));

        return false;
    }
}