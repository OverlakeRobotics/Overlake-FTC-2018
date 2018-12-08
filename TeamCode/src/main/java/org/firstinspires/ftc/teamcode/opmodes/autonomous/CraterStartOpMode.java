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

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

        ////
        waitForStart();
        ////

        // tensor flow
        int blockPos = determineBlockPos();
        driveSystem.driveToPositionInches(backCubeIn, -autonoPower, false);
        driveSystem.turnAbsolute(cratApproachDeg0, autonoPower);
        distanceSystem.getCloseToWall(cratTargDist1, toWallPow);
        //driveSystem.turnAbsolute(cratApproachDeg1, 1);
        //distanceSystem.getCloseToWall(7, 0.8);
        driveSystem.turnAbsolute(cratApproachDeg1, autonoPower);
        parkInDepot(-autonoPower, colorSystem);
        markerSystem.reset();
        sleep(300);
        parkOnCrator(autonoPower, initPitch, initRoll);
        markerSystem.reset();


        stop();
    }
}
