package org.firstinspires.ftc.teamcode.systems.logging;

import java.util.Date;

public class LoggingEntry
{
    private String tag;
    private LoggingLevel level;
    private String message;
    private long date;

    public LoggingEntry(String tag, LoggingLevel level, String message) {
        this.tag = tag;
        this.level = level;
        this.message = message;
        this.date = new Date().getTime();
    }
}
