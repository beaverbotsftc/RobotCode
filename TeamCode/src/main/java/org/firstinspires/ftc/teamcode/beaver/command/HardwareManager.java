package org.firstinspires.ftc.teamcode.beaver.command;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

public final class HardwareManager {
    private static final Set<String> claimedHardwareIds = new HashSet<>();
    private static HardwareMap hardwareMap = null;

    private HardwareManager() {}

    public static void reset() {
        claimedHardwareIds.clear();
        hardwareMap = null;
    }

    public static void init(HardwareMap hardwareMap) {
        reset();
        HardwareManager.hardwareMap = hardwareMap;
    }

    public static <T extends HardwareDevice> T get(String id) {
        //noinspection unchecked
        return (T) hardwareMap.get(id);
    }

    public static <T extends HardwareDevice> T get(Class<? extends T> classOrInterface, String id) {
        return hardwareMap.get(classOrInterface, id);
    }

    public static <T extends HardwareDevice> T claim(String id) {
        if (claimedHardwareIds.contains(id))
            throw new AlreadyClaimedException(String.format("Hardware with ID: '%s' already claimed, cannot claim it again.", id));
        claimedHardwareIds.add(id);
        return get(id);
    }

    public static <T extends HardwareDevice> T claim(Class<? extends T> classOrInterface, String id) {
        if (claimedHardwareIds.contains(id))
            throw new AlreadyClaimedException(String.format("Hardware with ID: '%s' already claimed, cannot claim it again.", id));
        claimedHardwareIds.add(id);
        return get(classOrInterface, id);
    }
}