package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CraterStartOpMode", group = "Bot")
public class CraterStartOpMode extends BaseAutonomousOpMode {
    private final String TAG = "CraterStartOpMode";

    public CraterStartOpMode () {
        super("CraterStartOpMode");
    }

    @Override
    public void runOpMode() {

        this.initSystems();
        waitForStart();
        delatch();
        sample();
        distanceSystem.getCloseToWall(cratTargDist1, powerToWall);
        driveSystem.turnAbsolute(cratApproachDeg1, autonomousPower);
        parkInDepot(-autonomousPower, colorSystem);
        markerSystem.place();
        sleep(3000);
        markerSystem.reset();
        parkOnCrator(autonomousPower, initPitch, initRoll);
        stop();
    }
}
