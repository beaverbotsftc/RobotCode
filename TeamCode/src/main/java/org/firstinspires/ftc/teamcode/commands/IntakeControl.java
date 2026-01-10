package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.util.Geometry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Led;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.List;

public class IntakeControl implements Command {
    private Intake intake;
    private Stopper stopper;
    private ColorSensor colorSensor;
    private Led led;
    private Gamepad gamepad;
    private Localizer localizer;
    private boolean restrict;
    private Side side;

    public IntakeControl(Intake intake, Stopper stopper, Localizer localizer, boolean restrict, Side side, ColorSensor colorSensor, Led led, Gamepad gamepad) {
        this.intake = intake;
        this.stopper = stopper;
        this.colorSensor = colorSensor;
        this.led = led;
        this.gamepad = gamepad;
        this.localizer = localizer;
        this.restrict = restrict;
        this.side = side;
    }

    public boolean periodic() {
        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        Pair<List<Double>, List<Double>> robot = Geometry.generateBox(localizer.getPosition().getX(), localizer.getPosition().getY(), Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH, localizer.getPosition().getTheta());

        if (gamepad.getRightBumper()
                && (!restrict ||
                Geometry.polygonPolygonIntersects(Constants.SHOOTING_ZONE_NEAR_X, Constants.SHOOTING_ZONE_NEAR_Y, robot.first, robot.second)
                || Geometry.polygonPolygonIntersects(Constants.SHOOTING_ZONE_FAR_X, Constants.SHOOTING_ZONE_FAR_Y, robot.first, robot.second))
        ) { // If in the shooting zone
            DrivetrainState goalPosition = new DrivetrainState(Constants.GOAL_X, side == Side.RED ? Constants.GOAL_Y : -Constants.GOAL_Y, 0);
            DrivetrainState position = localizer.getPosition();
            double distanceToGoal = goalPosition.lateralDistance(position);
            if (Math.abs(Localizer.wind(position.angleTo(goalPosition), localizer.getPosition().getTheta()) - Constants.shooterBias - localizer.getPosition().getTheta()) <
                    Math.toRadians(10) / distanceToGoal * 20 || !restrict
            ) {
                intake.setMaxPower(0.95);
                stopper.setMaxPower(0.95);
                if (ShooterControl.percentError < 7.0 && ShooterControl.percentError >= 0.0) {
                    intakeSpeed = 1.0;
                    stopper.spinForward();
                } else {
                    intakeSpeed = 0.0;
                    stopper.stop();
                }
            }
        } else if (intakeSpeed != 0) { //when intaking balls
            intake.setMaxPower(1);
            stopper.setMaxPower(0.5);

            stopper.spinReverse();
        } else {
            stopper.stop();
        }

        if (colorSensor.hasThreeBalls()) {
            led.setBallLedPurple();
        } else {
            led.turnOffBallLed();
        }

        intake.spin(intakeSpeed);

        return false;
    }
}
