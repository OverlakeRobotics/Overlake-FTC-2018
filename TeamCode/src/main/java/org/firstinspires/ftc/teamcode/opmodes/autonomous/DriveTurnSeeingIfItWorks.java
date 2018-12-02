package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;

public class DriveTurnSeeingIfItWorks extends LinearOpMode {

    private MecanumDriveSystem driveSystem;
    private int DRIVE_TO_POSITION = 34;

    @Override
    public void runOpMode() {
        init();
        waitForStart();
    }

    public void testTurn(){
        driveSystem.driveToPositionInches(34, 0.8);
        driveSystem.turn(90, 0.8);
    }
}
