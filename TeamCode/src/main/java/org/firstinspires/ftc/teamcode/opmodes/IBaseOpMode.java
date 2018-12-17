package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.logging.ILogger;

public interface IBaseOpMode
{
    ILogger getLogger();
    HardwareMap getHardwareMap();
    Telemetry getTelemetry();
}
