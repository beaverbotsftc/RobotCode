package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Set;

public final class SimpleControl implements Command {
    Gamepad gamepad;
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    double shootrpm = 2050.0;

    public SimpleControl(Gamepad gamepad, Drivetrain drivetrain, Intake intake, Shooter shooter) {
        this.gamepad = gamepad;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.shooter = shooter;
        this.drivetrain.setBrake(true);
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain, intake, shooter);
    }

    @Override
    public boolean periodic() {
        double x = changeInput(gamepad.getLeftY()) * 48;
        double y = -changeInput(gamepad.getLeftX()) * 48;
        double theta = -changeTurn(gamepad.getRightX()) * Math.PI * 2;

        drivetrain.move(new DrivetrainState(x, y, theta));

        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        intake.spin(intakeSpeed);

        if (gamepad.getDpadLeft()) {
            shooter.spin(shootrpm);

            double velocity = shooter.getVelocity();
            if (Math.abs(velocity - shootrpm) / shootrpm <= 0.05) {
                gamepad.rumble(0.45, 0.45, com.qualcomm.robotcore.hardware.Gamepad.RUMBLE_DURATION_CONTINUOUS);
            } else {
                gamepad.stopRumble();
            }
        } else {
            shooter.spin(0);
        }

        if (gamepad.getTriangle() && shootrpm < 4000) {
            shootrpm += 5;
        }
        if (gamepad.getCross() && shootrpm > 100) {
            shootrpm -= 5;
        }

        if (gamepad.getLeftBumperJustPressed()) {
            shooter.setHood(0.0);
            shootrpm = 2050.0;
        } else if (gamepad.getRightBumperJustPressed()) {
            shooter.setHood(0.51);
            shootrpm = 3000.0;
        }

        return false;
    }

    private double changeInput(double x) {
        return Math.signum(x) * (1 - Math.cos(x * Math.PI / 2.0));
    }

    private double changeTurn(double x) {
        return 4.0 * (Math.pow(x, 5)) / 5.0 + x / 5.0;
    }
}
