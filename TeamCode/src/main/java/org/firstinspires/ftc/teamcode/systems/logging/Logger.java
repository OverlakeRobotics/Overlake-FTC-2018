package org.firstinspires.ftc.teamcode.systems.logging;

public abstract class Logger implements ILogger
{
    @Override
    public void error(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Error, data, args);
    }

    @Override
    public void warn(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Warn, data, args);
    }

    @Override
    public void info(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Info, data, args);
    }

    @Override
    public void verbose(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Verbose, data, args);
    }

    @Override
    public void debug(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Debug, data, args);
    }

    @Override
    public void silly(String tag, Object data, Object... args)
    {
        log(tag, LoggingLevel.Silly, data, args);
    }

    public abstract void log(String tag, LoggingLevel level, Object data, Object... args);
}
