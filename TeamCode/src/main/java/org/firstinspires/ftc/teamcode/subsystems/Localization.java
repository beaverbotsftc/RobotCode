package org.firstinspires.ftc.teamcode.subsystems;

import org.beaverbots.BeaverCommand.Subsystem;

import java.util.Set;

public class Localization implements Subsystem {
    private Pinpoint pinpoint;
    private Limelight limelight;

    public Set<Subsystem> getDependencies() {
        return Set.of(limelight);
    }
    public Localization(Pinpoint pinpoint, Limelight limelight) {
        this.pinpoint = pinpoint;
        this.limelight = limelight;
    }
}
