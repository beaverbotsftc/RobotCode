package org.firstinspires.ftc.teamcode.tuning;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.CommandRuntimeOpMode;
import org.beaverbots.beaver.pathing.commands.HolonomicFollowPath;
import org.beaverbots.beaver.pathing.PIDF;
import org.beaverbots.beaver.pathing.PIDFAxis;
import org.beaverbots.beaver.pathing.Path;
import org.beaverbots.beaver.pathing.PathAxis;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class PTuningY extends CommandRuntimeOpMode {
    private Drivetrain drivetrain;
    private Pinpoint pinpoint;
    private Gamepad gamepad;

    private boolean updating = true;

    private double p = 0;

    private Command movement;

    @Override
    public void onInit() {
        drivetrain = new MecanumDrivetrain(1);
        pinpoint = new Pinpoint(new DrivetrainState(0, 0, 0));
        gamepad = new Gamepad(gamepad1);
    }

    @Override
    public void onStart() {
        register(drivetrain, pinpoint, gamepad);
    }

    @Override
    public void periodic() {
        telemetry.addData("P:", p);
        telemetry.addData("Updating", updating);
        telemetry.update();
        if (gamepad.getCrossJustPressed()) {
            updating = !updating;
            if (updating) {
                cancel(movement);
                drivetrain.move(new DrivetrainState(0, 0, 0));
            } else {
                movement = new HolonomicFollowPath(
                        new Path(
                                List.of(
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY),
                                        new PathAxis(t -> 0, 0, Double.POSITIVE_INFINITY)
                                ),
                                t -> false
                        ),
                        new PIDF(
                                List.of(
                                        new PIDFAxis(new PIDFAxis.K(0, 0, 0, 0, 0, 0, 0, 0)),
                                        new PIDFAxis(new PIDFAxis.K(p, 0, 0, 0, 0, Double.POSITIVE_INFINITY, 0, 0)),
                                        new PIDFAxis(new PIDFAxis.K(0, 0, 0, 0, 0, 0, 0, 0))
                                )
                        ),
                        pinpoint,
                        drivetrain
                );
                schedule(movement);
            }
        }

        if (updating) {
            p += gamepad.getRightX() / 100;
        }
    }
}
