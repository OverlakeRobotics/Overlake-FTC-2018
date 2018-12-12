package org.firstinspires.ftc.teamcode.systems.logging;

public interface ILogger
{
    void error(String tag, Object data, Object... args);
    void warn(String tag, Object data, Object... args);
    void info(String tag, Object data, Object... args);
    void verbose(String tag, Object data, Object... args);
    void debug(String tag, Object data, Object... args);
    void silly(String tag, Object data, Object... args);
    void close();
}
