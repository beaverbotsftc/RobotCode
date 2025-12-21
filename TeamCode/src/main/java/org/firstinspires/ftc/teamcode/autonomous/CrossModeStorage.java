package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Side;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainState;

public class CrossModeStorage {
    public static DrivetrainState position = new DrivetrainState(0, 0, 0);
    public static Side side = Side.RED;
}
