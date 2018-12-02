package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;

public class DriveTurnSeeingIfItWorks extends LinearOpMode {

    private MecanumDriveSystem driveSystem;

    @Override
    public void runOpMode() {
        init();
        waitForStart();
    }

    public void testTurn(){
        driveSystem.turn(90);
    }
}
