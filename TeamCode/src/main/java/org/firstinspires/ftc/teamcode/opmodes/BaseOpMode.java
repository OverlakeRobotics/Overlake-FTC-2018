package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.logging.ILogger;
import org.firstinspires.ftc.teamcode.systems.logging.RobotHubLogger;

public abstract class BaseOpMode extends OpMode implements IBaseOpMode
{
    public ILogger logger;
    public Telemetry telemetry;

    public BaseOpMode() {
        this.logger = new RobotHubLogger();
        this.telemetry = super.telemetry;
    }

    /**
     * Called by the opmode when it is stopped, can be overridden by other opmodes
     */
    public void close() { }

    /**
     * Called when opmode is stopped
     */
    @Override
    public void stop() {
        logger.close();
        close();
    }

    public ILogger getLogger()
    {
        return logger;
    }

    public HardwareMap getHardwareMap()
    {
        return hardwareMap;
    }

    public Telemetry getTelemetry() {
        return this.telemetry;
    }
}
