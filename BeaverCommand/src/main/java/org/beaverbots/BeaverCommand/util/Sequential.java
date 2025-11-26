package org.beaverbots.BeaverCommand;

public class SequentialCommandGroup extends CommandGroup {
    private boolean newCommandStart = true;

    @Override
    public boolean periodic() {
        if (newCommandStart)
            commands.get(0).start();

        if (commands.get(0).periodic()) {
            commands.get(0).stop();
            commands.remove(0);
            newCommandStart = true;
        }

        return commands.isEmpty();
    }

    @Override
    public void stop() {
        if (!commands.isEmpty())
            commands.get(0).stop();
    }
}
