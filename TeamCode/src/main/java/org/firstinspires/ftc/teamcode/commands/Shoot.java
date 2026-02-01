package org.firstinspires.ftc.teamcode.commands;

import android.util.Pair;

import org.beaverbots.beaver.command.Command;
import org.beaverbots.beaver.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.Transform;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.TurretV2;
import org.firstinspires.ftc.teamcode.subsystems.localizer.Localizer;

import java.util.Set;
import java.util.function.BooleanSupplier;

public class Shoot implements Command {
    private Intake intake;
    private Stopper stopper;
    private BooleanSupplier isReady;

    public Shoot(Intake intake, Stopper stopper, BooleanSupplier isReady) {
        this.intake = intake;
        this.stopper = stopper;
        this.isReady = isReady;
    }

    public Set<Subsystem> getDependencies() {
        return Set.of(intake, stopper);
    }

    public boolean periodic() {
        if (isReady.getAsBoolean()) {
            intake.spin(1);
            stopper.spin(1);
            intake.empty();
        } else {
            intake.spin(0);
            stopper.spin(0);
        }

        return false;
    }
}
