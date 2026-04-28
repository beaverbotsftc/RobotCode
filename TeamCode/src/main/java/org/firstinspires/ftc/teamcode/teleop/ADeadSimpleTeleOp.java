package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.beaverbots.beaver.command.CommandOpMode;
import org.beaverbots.beaver.command.premade.Repeat;
import org.beaverbots.beaver.util.Transform;
import org.firstinspires.ftc.teamcode.subsystems.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp
public class ADeadSimpleTeleOp extends CommandOpMode {
    private VoltageSensor voltageSensor;
    private Drivetrain drivetrain;
    private Intake intake;
    private Turret turret;
    private Pinpoint pinpoint;

    private GamepadEx gamepad;

    public void onInit() {
        voltageSensor = new VoltageSensor();
        drivetrain = new SwerveDrivetrain(voltageSensor);
        intake = new Intake();
        turret = new Turret(voltageSensor);

        pinpoint = new Pinpoint(Transform.ZERO);

        gamepad = new GamepadEx(gamepad1);

        register(voltageSensor, pinpoint, gamepad);
    }

    double rpm = 0;
    double hood = 0;

    public void onStart() {
        register(drivetrain, intake, turret);
        turret.turn(0);
        schedule(
                new Repeat(() -> drivetrain.move(new Transform(gamepad.getLeftY(), -gamepad.getLeftX(), -gamepad.getRightX()))),
                new Repeat(() -> intake.intake(gamepad.getRightTrigger() - gamepad.getLeftTrigger())),
                new Repeat(() -> intake.transfer(gamepad.getRightBumper())),
                new Repeat(() -> turret.shoot(rpm)),
                new Repeat(() -> turret.setHoodAngle(hood))
        );
    }

    public void periodic() {
        if (gamepad.getDpadUpJustPressed()) rpm += 200;
        if (gamepad.getDpadDownJustPressed()) rpm -= 200;
        if (gamepad.getDpadLeftJustPressed()) hood -= 0.1;
        if (gamepad.getDpadRightJustPressed()) hood += 0.1;

        addData("rpm", rpm);
        addData("actual rpm", turret.getVelocity());
        addData("hood", hood);

        addData("Pos", pinpoint.getPosition());
    }
}
