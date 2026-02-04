package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

public class ShooterControl2 implements Command {
    private Gamepad gamepad;
    private Shooter shooter;
    private Intake intake;
    private Stopper stopper;
    private Localizer localizer;
    private Side side;

    private Transform goalPosition;

    public ShooterControl2(Shooter shooter, Intake intake, Stopper stopper, Localizer localizer, Side side, Gamepad gamepad) {
        this.shooter = shooter;
        this.localizer = localizer;
        this.gamepad = gamepad;
        this.side = side;
        this.stopper = stopper;
        this.intake = intake;
        this.side = side;

        goalPosition = new Transform(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0);
    }

    public boolean periodic() {
        double distanceToGoal = localizer.getPosition().lateralDistance(goalPosition);
        double relativeAngleToGoal = localizer.getPosition().relativeAngleTo(goalPosition);

        Pair<Double, Double> values = Shooter.getSettingsAtDistance(distanceToGoal);

        shooter.setFlywheelSpeed(values.first);
        shooter.setHoodAngle(values.second);

        if (gamepad.getRightBumper() && Math.abs(relativeAngleToGoal) / distanceToGoal < Constants.maxAngularShootingError) {
            intake.spin(1);
            stopper.spin(1);
        } else {
            intake.spin(0);
            stopper.spin(0);
        }

        return false;
    }
}
