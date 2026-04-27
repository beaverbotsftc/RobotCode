package org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.Subsystem;
import org.beaverbots.beaver.pidf.PIDFAxis;
import org.beaverbots.beaver.util.Geometry;
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

    private PIDFAxis pidfGate;
    private PIDFAxis pidfNormal;

    private Stopwatch stopwatch;

    ///  Note that this isn't the standard field mirroring, the symmetries are different.
    /// x -> -x, y -> -y, theta -> theta
    public SwerveDriveControl(SwerveDrivetrain drivetrain, Localizer localizer, GamepadEx gamepad, DoubleUnaryOperator[] mirror) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;

        this.gamepad = gamepad;
        this.mirror = mirror;

        pidfGate = new PIDFAxis(new PIDFAxis.K(Constants.pidPHeadingEnforcement, Constants.pidIHeadingEnforcement, Constants.pidDHeadingEnforcement, new double[]{}, 1, 1, Constants.pidTauHeadingEnforcement, Constants.pidGammaHeadingEnforcement, 0.1));
        pidfNormal = new PIDFAxis(new PIDFAxis.K(Constants.pidPGateHeading, Constants.pidIGateHeading, Constants.pidDGateHeading, new double[]{}, 1, 1, Constants.pidTauGateHeading, Constants.pidGammaGateHeading, 0.1));

        stopwatch = new Stopwatch();
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    @Override
    public void start() {
        targetHeading = localizer.getPosition().getTheta();
        pidfGate.reset();
        stopwatch.reset();
    }

    @Override
    public boolean periodic() {
        CommandOpMode.addData("Follow Target Heading", followTargetHeading);
        CommandOpMode.addData("Heading lock", gamepad.getAPressedToggle());

        double scale = gamepad.getLeftStickPressed() ? 0.5 : 1;

        if (gamepad.getX()) {
            drivetrain.x();
        } else if (gamepad.getAPressedToggle()) {
            double heading = localizer.getPosition().getTheta();
            double gateHeading = Geometry.unnormalizeAngle(mirror[2].applyAsDouble(2.23), heading);
            double control = -pidfGate.update(heading - gateHeading, new double[]{}, stopwatch.getDt());

            drivetrain.move(new Transform(gamepad.getLeftX(), gamepad.getLeftY(), control).transform(mirror).toLocalVelocity(localizer.getPosition()).scale(scale));
        } else {
            if (gamepad.getRightX() != 0 || new Transform(gamepad.getLeftX(), gamepad.getLeftY()).lateralNorm() < Constants.headingEnforcementLateralForceCutoff)
                followTargetHeading = false;
            else if (!followTargetHeading && Math.abs(localizer.getVelocity().getTheta()) < Constants.headingEnforcementAngularVelocityCutoff) {
                followTargetHeading = true;
                targetHeading = localizer.getPosition().getTheta();

                pidfNormal.reset();
            }

            if (followTargetHeading) {
                double heading = localizer.getPosition().getTheta();
                double control = -pidfNormal.update(heading - targetHeading, new double[]{}, stopwatch.getDt());

                drivetrain.move(new Transform(gamepad.getLeftX(), gamepad.getLeftY(), control).transform(mirror).toLocalVelocity(localizer.getPosition()).scale(scale));
            } else
                drivetrain.move(new Transform(gamepad.getLeftX(), gamepad.getLeftY(), -Math.pow(gamepad.getRightX(), 3)).transform(mirror).toLocalVelocity(localizer.getPosition()).scale(scale));
        }

        return false;
    }
}