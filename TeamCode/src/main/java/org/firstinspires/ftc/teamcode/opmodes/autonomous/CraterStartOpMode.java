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
        if (blockPos == 1) {
            driveSystem.driveToPositionInches(cratTargDist3, -autonoPower, false);
        } else if (blockPos == 2) {
            driveSystem.driveToPositionInches(cratTargDist4, -autonoPower, false);
        } else {
            driveSystem.driveToPositionInches(cratTargDist2, -autonoPower, false);
        }
        //distanceSystem.getCloseToWall(cratTargDist1, toWallPow);
        distanceSystem.strafeTowardWall(cratTargDist1, cratApproachDeg1, toWallPow);
        //driveSystem.turnAbsolute(cratApproachDeg1, autonoPower);
        parkInDepot(-autonoPower, colorSystem);
        markerSystem.reset();
        sleep(300);
        if (zone == 0) {
            distanceSystem.driveAlongWallInches(40, 2, 5, autonoPower,false);
        } else if (zone == 1) {
            driveSystem.driveToPositionInches(35, autonoPower, false);
            distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        }
        parkOnCrator(autonoPower, initPitch, initRoll);
        markerSystem.reset();


        stop();
    }
}
