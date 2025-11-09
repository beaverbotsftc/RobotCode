package org.firstinspires.ftc.teamcode.commands;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.DrivetrainState;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Set;

public final class DirectControl implements Command {
    Gamepad gamepad;
    Drivetrain drivetrain;
    Intake intake;
    public DirectControl(Gamepad gamepad, Drivetrain drivetrain, Intake intake) {
        this.gamepad = gamepad;
        this.drivetrain = drivetrain;
        this.intake = intake;
    }

    @Override
    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain, intake);
    }

    @Override
    public boolean periodic() {
        double x = gamepad.getLeftY();
        double y = -gamepad.getLeftX();
        double theta = -gamepad.getRightX();

        drivetrain.move(new DrivetrainState(x, y, theta));

        double intakeSpeed = gamepad.getRightTrigger() - gamepad.getLeftTrigger();

        intake.spin(intakeSpeed);

        return false;
    }
}
