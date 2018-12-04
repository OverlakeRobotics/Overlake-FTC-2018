package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.configs.ConfigParser;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.distance.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;
import org.firstinspires.ftc.teamcode.systems.marker.Marker;

/**
 * Created by EvanCoulson on 10/11/17.
 */

public abstract class BaseAutonomousOpMode extends LinearOpMode
{
    ConfigParser config;
    public MecanumDriveSystem driveSystem;
    public IMUSystem imuSystem;
    public ColorSystem colorSystem;
    public DistanceSystem distanceSystem;
    public Marker markerSystem;

    public double initPitch;
    public double initRoll;

    public int zone;
    public int backCubeIn;
    public int approachDeg0;
    public int approachDeg1;
    public int approachDeg2;
    public double targDist1;

    public BaseAutonomousOpMode(String opModeName)
    {
        //config = new ConfigParser(opModeName + ".omc");
        config = new ConfigParser("Autonomous.omc");
        telemetry.setMsTransmissionInterval(200);
    }

    protected void initSystems()
    {
        this.driveSystem = new MecanumDriveSystem(this);
        this.imuSystem = new IMUSystem(this);
        colorSystem = new ColorSystem(this);
        distanceSystem = new DistanceSystem(this, driveSystem, colorSystem);
        markerSystem = new Marker(this);

        zone = config.getInt("zone");
        backCubeIn = config.getInt("backCubeIn");//10
        approachDeg0 = config.getInt("approachDeg0");//-90
        approachDeg1 = config.getInt("approachDeg1");//-120
        config.getInt("approachDeg2");//-135
        config.getDouble("targDist1");

        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();
    }

    public void parkInDepot(double maxPower, ColorSystem colorSystem) {
        int redTriggerValue = 12;
        int blueTriggerValue = 8;
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while ((colorSystem.getRed() < redTriggerValue) &&
                (colorSystem.getBlue() < blueTriggerValue)) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void parkOnCrator(double maxPower, double initPitch, double initRoll) {
        double criticalAngle = 1.5;
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(maxPower);

        while (((Math.abs(imuSystem.getPitch() - initPitch) < criticalAngle) ||
                (Math.abs(imuSystem.getRoll() - initRoll) < criticalAngle))) {
            driveSystem.setPower(maxPower);
        }
        driveSystem.setPower(0);
    }

    public void telem(String message, double sec) {
        telemetry.addLine(message);
        telemetry.update();
        sleep((int)(1000 * sec));
    }
}
