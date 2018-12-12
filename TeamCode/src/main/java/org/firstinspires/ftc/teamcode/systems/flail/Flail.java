package org.firstinspires.ftc.teamcode.systems.flail;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.System;

/**
 * Flail system runs a flail and stops a flail
 */
public class Flail extends System {
    private DcMotor flailMotor;

    /**
     * Creates a new flail object in the current opmode
     * @param opMode uses the current opmode to build a new flail
     */
    public Flail(IBaseOpMode opMode) {
        super (opMode,"flail");
        flailMotor = hardwareMap.dcMotor.get("flail");
    }

    /**
     * Starts the flail
     */
    public void runForward() {
        flailMotor.setPower(-1);
    }

    public void runBackward() {
        flailMotor.setPower(1);
    }

    /**
     * Stops the flail
     */
    public void stop() {
        flailMotor.setPower(0);
    }
}

