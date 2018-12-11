package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

@Autonomous(name = "DepotStartOpMode", group = "Bot")
public class DepotStartOpMode extends BaseAutonomousOpMode {
    private final String TAG = "DepotStartOpMode";

    public int cubePos;

    public DepotStartOpMode() {
        super("DepotStartOpMode");

        cubePos = config.getInt("cubePos");
    }

    @Override
    public void runOpMode() {

        telem("About to initialize systems.", 0.25);
        this.initSystems();
        telem("Initialized all systems. Ready." + ("" + distanceSystem), 0.25);

        ////
        waitForStart();
        sample();
        driveSystem.driveToPositionInches(backCubeIn, -1, false);
        driveSystem.turnAbsolute(cratApproachDeg0, 1);
        distanceSystem.getCloseToWall(cratTargDist1, 1);
        driveSystem.turnAbsolute(approachDeg2, 1);
        parkInDepot(1, colorSystem);
        markerSystem.place();
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.BACKWARD);
        distanceSystem.driveAlongWallInches(40, 3, 6, 1, true);
        parkOnCrator(1, initPitch, initRoll);

        stop();
    }
}
