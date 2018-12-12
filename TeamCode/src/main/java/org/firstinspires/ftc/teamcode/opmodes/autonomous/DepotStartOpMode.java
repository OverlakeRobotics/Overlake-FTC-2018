package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;

@Autonomous(name = "DepotStartOpMode", group = "Bot")
public class DepotStartOpMode extends BaseAutonomousOpMode {
    private final String TAG = "DepotStartOpMode";


    public DepotStartOpMode() {
        super("DepotStartOpMode");



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
        parkInDepot(autonoPower, colorSystem);
        markerSystem.reset();
        sleep(400);
        distanceSystem.strafeTowardWall(depDepappraochIn, depWallHeading, autonoPower);
        distanceSystem.driveAlongWallInches(depToCratIn, 1, 3, autonoPower, true);
        parkOnCrator(autonoPower, CRITICAL_ANGLE, CRITICAL_ANGLE);

        stop();
    }
}
