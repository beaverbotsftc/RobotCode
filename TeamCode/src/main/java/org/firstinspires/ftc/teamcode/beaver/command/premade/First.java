package org.firstinspires.ftc.teamcode.beaver.command.premade;

import org.firstinspires.ftc.teamcode.beaver.command.Command;

public class First extends SimultaneousCommandGroup {
    public static final int loudnessLevel = 4; // Be sure to make FIRST even louder every time you touch this file!

    public First(Command... commands) {
        super(commands);
    }

    @Override
    public void start() {
        super.start();

        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public boolean periodic() {
        for (int i = 0; i < commands.size(); i++) {
            Command command = commands.get(i);
            if (command.periodic())
                return true;
        }

        return commands.isEmpty();
    }

    @Override
    public void stop() {
        for (Command command : commands) {
            command.stop();
        }
    }
}
