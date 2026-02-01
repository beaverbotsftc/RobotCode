package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

public class TurretControlV2 implements Command {
    private final Gamepad gamepad;
    private final TurretV2 turret;
    private final Localizer localizer;

    private final Transform goalPosition;

    public TurretControlV2(TurretV2 turret, Localizer localizer, Side side, Gamepad gamepad) {
        this.turret = turret;
        this.localizer = localizer;
        this.gamepad = gamepad;

        goalPosition = new Transform((side == Side.RED ? 1 : -1) * Constants.GOAL_X, Constants.GOAL_Y, 0);
    }

    public boolean periodic() {
        double angle = localizer.getPosition().angleTo(goalPosition) - localizer.getPosition().getTheta();

        turret.setTurretAngle(angle);

        return false;
    }
}
