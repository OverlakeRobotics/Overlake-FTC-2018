package org.firstinspires.ftc.teamcode.systems.logging;

public class RobotHubLogger extends Logger
{
    RobotHubLoggingClient robotHubLoggingClient;

    public RobotHubLogger() {
        robotHubLoggingClient = new RobotHubLoggingClient();
        Thread robotHubLoggingThread = new Thread(robotHubLoggingClient);
        robotHubLoggingThread.start();
    }

    @Override
    public void log(String tag, LoggingLevel level, Object data, Object... args)
    {
        robotHubLoggingClient.addLoggingEntry(new LoggingEntry(tag, level, StringFormatter.format(data.toString(), args)));
    }

    public void stop() {
        robotHubLoggingClient.stop();
    }
}
