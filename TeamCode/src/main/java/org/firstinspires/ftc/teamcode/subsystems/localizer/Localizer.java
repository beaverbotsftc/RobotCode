package org.firstinspires.ftc.teamcode.subsystems.localizer;

import org.beaverbots.beaver.util.Transform;

public interface Localizer extends org.beaverbots.beaver.pathing.Localizer {
    Transform getPosition();
    Transform getVelocity();
}
