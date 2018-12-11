package org.firstinspires.ftc.teamcode.systems.logging;

import android.os.Environment;
import android.util.Log;

import java.io.File;

/**
 * A logger that logs to a file in the robot app directory
 */
public class FileLogger extends Logger
{
    private LogWritter logWritter;
    private File logFile;

    /**
     * Creates a new File Logger at a given path
     * @param path the path of the file to be logged to
     */
    public FileLogger(String path) {
        logFile = new File(Environment.getExternalStorageDirectory() + path);
        logWritter = new LogWritter(logFile);
    }

    /**
     * Logs to the file that holds logging data
     * @param name the key of the log on the file
     * @param level the level of the log entry
     * @param data the raw string to be formatted
     * @param args the arguments to be put into the formatted string
     */
    public void log(String name, LoggingLevel level, Object data, Object... args) {
        String line = StringFormatter.format("[{0}] {2}: {1}", name, StringFormatter.format(data.toString(), args), level);
        logWritter.appendLine(line);
        logWritter.flush();
    }
}
