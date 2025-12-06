package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.beaverbots.BeaverCommand.Command;
import org.beaverbots.BeaverCommand.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

import java.util.Set;


public class DrivetrainControl implements Command {
    private Drivetrain drivetrain;
    private Gamepad gamepad;

    public DrivetrainControl(Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(drivetrain);
    }

    @Override
    public boolean periodic() {
        double x = gamepad.getLeftY() / Constants.drivetrainPowerConversionFactorX; //* 70;
        double y = -gamepad.getLeftX() / Constants.drivetrainPowerConversionFactorY; //* 70;
        double theta = -gamepad.getRightX() / Constants.drivetrainPowerConversionFactorTheta; //* Math.PI * 2;

        drivetrain.move(new DrivetrainState(x, y, theta));

        if (gamepad.getDpadDownJustPressed()) {
            drivetrain.toggleBrake();
        }

        return false;
    }
}
