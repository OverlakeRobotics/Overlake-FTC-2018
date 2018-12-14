package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.logging.ILogger;

public interface IBaseOpMode
{
    ILogger getLogger();
    HardwareMap getHardwareMap();
}
