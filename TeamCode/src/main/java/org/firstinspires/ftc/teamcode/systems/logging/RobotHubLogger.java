package org.firstinspires.ftc.teamcode.systems.logging;

public class RobotHubLogger extends Logger
{
    Thread robotHubLoggingThread;
    RobotHubLoggingClient robotHubLoggingClient;

    public RobotHubLogger() {
        robotHubLoggingClient = new RobotHubLoggingClient();
        robotHubLoggingThread = new Thread(robotHubLoggingClient);
        robotHubLoggingThread.start();
    }

    @Override
    public void log(String tag, LoggingLevel level, Object data, Object... args)
    {
        robotHubLoggingClient.addLoggingEntry(new LoggingEntry(tag, level, StringFormatter.format(data.toString(), args)));
    }
}
