package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
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

    public SimpleControl(Gamepad gamepad, Drivetrain drivetrain, Intake intake, Shooter shooter) {
        this.gamepad = gamepad;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain, intake, shooter);
    }

    @Override
    public boolean periodic() {
        double x = gamepad.getLeftY() * 48;
        double y = -gamepad.getLeftX() * 48;
        double theta = -gamepad.getRightX() * Math.PI * 2;

        drivetrain.move(new DrivetrainState(x, y, theta));

        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        intake.spin(intakeSpeed);

        double shooterPower = gamepad.getRightY();

        shooter.spin(shooterPower);

        return false;
    }
}
