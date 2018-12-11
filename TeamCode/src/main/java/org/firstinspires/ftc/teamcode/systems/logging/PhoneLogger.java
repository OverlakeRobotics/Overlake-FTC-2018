package org.firstinspires.ftc.teamcode.systems.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry.Item;

import java.util.HashMap;

/**
 * A logger that build on top of the log provided by the ftc core library
 */
public class PhoneLogger extends Logger
{
    private Telemetry telemetry;

    /**
     * Creates a logger to be used on the phone
     * @param telemetry log provided by the opmode
     */
    public PhoneLogger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Logs to phone
     * @param tag tag to log on the phone
     * @param level level is ignored on the phone's log
     * @param data data to be logged
     * @param args arguments to be used when logging
     */
    @Override
    public void log(String tag, LoggingLevel level, Object data, Object... args)
    {
        telemetry.addData(tag, StringFormatter.format(data.toString(), args));
        telemetry.update();
    }
}
