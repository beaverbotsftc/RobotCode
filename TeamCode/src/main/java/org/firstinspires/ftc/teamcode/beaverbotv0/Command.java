package org.firstinspires.ftc.teamcode.beaverbotv0;

import android.media.ResourceBusyException;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public abstract class Command {
    private static Set<Subsystem> reserved = new HashSet<>();

    private Set<Subsystem> requirements;

    public Command(Set<Subsystem> requirements) throws ResourceBusyException {
        this.requirements = requirements;

        if (!Collections.disjoint(requirements, reserved)) {
            throw new ResourceBusyException(String.format("Requirements are not disjoint with reserved resources : %s required and %s reserved.", requirements, reserved));
        }

        reserved.addAll(requirements);
    }

    abstract void init();

    abstract boolean tick();
}