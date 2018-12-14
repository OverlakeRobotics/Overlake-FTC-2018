package org.firstinspires.ftc.teamcode.systems.logging;

public class RobotHubLogger extends Logger
{
    RobotHubLoggingClient robotHubLoggingClient;
    boolean hasClosed;

    public RobotHubLogger() {
        robotHubLoggingClient = new RobotHubLoggingClient();
        Thread robotHubLoggingThread = new Thread(robotHubLoggingClient);
        robotHubLoggingThread.start();
        hasClosed = false;
    }

    @Override
    public void log(String tag, LoggingLevel level, Object data, Object... args)
    {
        robotHubLoggingClient.addLoggingEntry(new LoggingEntry(tag, level, StringFormatter.format(data.toString(), args)));
    }

    public void close() {
        robotHubLoggingClient.addLoggingEntry(null);
    }

    protected void finalize() {
        if (!hasClosed) {
            this.close();
        }
    }
}
