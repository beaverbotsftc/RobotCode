package org.beaverbots.BeaverCommand;

public class SequentialCommandGroup extends CommandGroup {
    private int currentIndex = 0;

    public SequentialCommandGroup(Command... commands) {
        super(commands);
    }

    @Override
    public void start() {
        currentIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).start();
        }
    }

    @Override
    public boolean periodic() {
        if (commands.isEmpty())
            return true;

        Command currentCommand = commands.get(currentIndex);

        if (currentCommand.periodic()) {
            currentCommand.stop();
            currentIndex++;

            if (currentIndex < commands.size())
                commands.get(currentIndex).start();
            else
                return true;
        }

        return false;
    }

    @Override
    public void stop() {
        if (currentIndex < commands.size())
            commands.get(currentIndex).stop();
    }
}