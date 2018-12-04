package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.marker.Marker;
import org.firstinspires.ftc.teamcode.systems.tensorflow.TensorFlow;

@Autonomous(name = "Depot Start")
public class DepotOpMode extends LinearOpMode
{

    private MecanumDriveSystem driveSystem;
    private Marker markerServo;
    private ColorSystem colorSystem;

    @Override
    public void runOpMode() {
        initializeOpMode();
        waitForStart();
        driveSystem.parkInDepot(0.6, colorSystem);
        //driveSystem.driveToPositionInches(50, 0.2);
        markerServo.place();
        markerServo.reset();
    }
    private void initializeOpMode() {
        driveSystem = new MecanumDriveSystem(this);
        markerServo = new Marker(this);
        colorSystem = new ColorSystem(this);
    }
}