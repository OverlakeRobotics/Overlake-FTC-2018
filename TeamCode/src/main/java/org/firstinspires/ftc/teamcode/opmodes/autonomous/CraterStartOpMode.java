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
        driveSystem.driveToPositionInches(backCubeIn, -autonoPower, false);
        driveSystem.turnAbsolute(cratApproachDeg0, autonoPower);
        driveSystem.driveToPositionInches(cratTargDist3, -autonoPower, false);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        parkInDepot(-autonoPower, colorSystem);
        markerSystem.place();
        sleep(1000);
        markerSystem.reset();
        driveSystem.driveToPositionInches(50, autonoPower, false);
        distanceSystem.strafeTowardWall(inFromWall, cratApproachDeg1, autonoPower);
        parkOnCrator(toWallPow, initPitch, initRoll);
        markerSystem.reset();


        stop();
    }
}
