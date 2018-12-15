package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.systems.logging.ILogger;
import org.firstinspires.ftc.teamcode.systems.logging.RobotHubLogger;

public abstract class BaseLinearOpMode extends LinearOpMode implements IBaseOpMode
{
    ILogger logger;

    public BaseLinearOpMode() {
        logger = new RobotHubLogger();
    }

    public ILogger getLogger() {
        return logger;
    }

    public HardwareMap getHardwareMap() {
        return this.hardwareMap;
    }

    public void runOpMode() {
        run();
        logger.close();
    }

    public abstract void run();
}
