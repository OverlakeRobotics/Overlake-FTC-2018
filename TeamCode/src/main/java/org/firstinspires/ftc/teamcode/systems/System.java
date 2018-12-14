package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.logging.ILogger;


public abstract class System {

    protected String systemName;
    protected HardwareMap hardwareMap;
    public ILogger log;

    public System(IBaseOpMode opMode, String systemName) {
        this.systemName = systemName;
        this.hardwareMap = opMode.getHardwareMap();
        this.log = opMode.getLogger();
    }
}
