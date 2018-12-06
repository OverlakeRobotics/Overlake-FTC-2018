package org.firstinspires.ftc.teamcode.systems.lidar;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.base.System;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;

public class LidarNavigationSystem extends System {
    DistanceSensor lidar;
    DistanceSensor lidar2;
    Rev2mDistanceSensor sensorTimeOfFlight;
    HardwareMap hwmap;
    MecanumDriveSystem driveSystem;
    IMUSystem imu;
    ColorSystem colorSystem;
    OpMode opMode;

    double initPitch;
    double initRoll;

    public LidarNavigationSystem(OpMode opMode, MecanumDriveSystem driveSystem, ColorSystem colorSystem) {
        super(opMode, "DriveSystem4Wheel");
        this.hwmap = opMode.hardwareMap;
        this.driveSystem = driveSystem;
        imu = new IMUSystem(opMode);
        this.colorSystem = colorSystem;
        initSystem();

        initPitch = imu.getPitch();
        initRoll = imu.getRoll();
    }

    public void initSystem() {
        lidar = hwmap.get(DistanceSensor.class, "sensor_range");
        lidar2 = hwmap.get(DistanceSensor.class, "sensor_range2");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)lidar;
    }

    public double getDistance1() {
        return lidar.getDistance(DistanceUnit.INCH);
    }

    public double getDistance2() {
        return lidar2.getDistance(DistanceUnit.INCH);
    }

    public void telemetry() {
        telemetry.log("range1", String.format("%.01f in", lidar.getDistance(DistanceUnit.INCH)));
        telemetry.log("range2", String.format("%.01f in", lidar2.getDistance(DistanceUnit.INCH)));
        telemetry.log("frontRightDistance: ", "" + getFrontRightDistance());
        telemetry.write();
    }

    public double getFrontRightDistance() {
        double d1 = getDistance1();
        double d2 = getDistance2();
        double df = d1 - d2;
        double ds = 10.0; // inches
        double d = (d1 * (Math.sin(Math.atan((ds/df)))));
        return d;
    }

    public void driveAlongWallInches(int inches, double closeBuffer, double farBuffer, double power,
                                     boolean shouldRamp) {
        int ticks = driveSystem.inchesToTicks(inches);
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.motorFrontRight.setTargetPosition(driveSystem.motorFrontRight.getCurrentPosition() + ticks);
        driveSystem.motorFrontLeft.setTargetPosition(driveSystem.motorFrontLeft.getCurrentPosition() + ticks);
        driveSystem.motorBackRight.setTargetPosition(driveSystem.motorBackRight.getCurrentPosition() + ticks);
        driveSystem.motorBackLeft.setTargetPosition(driveSystem.motorBackLeft.getCurrentPosition() + ticks);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp ramp = new ExponentialRamp(new Point(0, driveSystem.RAMP_POWER_CUTOFF),
                new Point(driveSystem.RAMP_DISTANCE_TICKS, power));

        double adjustedPower = Range.clip(power, -1.0, 1.0);

        driveSystem.setPower(adjustedPower);

        while (driveSystem.anyMotorsBusy()) {
            int distance = driveSystem.getMinDistanceFromTarget();

            if (distance < 50) {
                break;
            }

            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.DISTANCE);

            double direction = 1.0;
            if (distance < 0) {
                distance = -distance;
                direction = -1.0;
            }

            double scaledPower = shouldRamp ? ramp.scaleX(distance) : power;
            telemetry.log("LidarNavigationSystem",
                    "ticks left (ticks): " + driveSystem.getMinDistanceFromTarget());
            telemetry.log("LidarNavigationSystem","scaled power: " + scaledPower);
            telemetry.log("LidarNavigationSystem", "distance1: " + getDistance1());
            telemetry.log("LidarNavigationSystem", "distance2: " + getDistance2());
            driveSystem.setPower(direction * scaledPower);
            telemetry.write();
        }
        driveSystem.setPower(0);
        telemetry.write();
    }

    public void correctToFollowWall(double closeBuffer, double farBuffer, double correctionPower,
                                    StopCondition stopCondition) {
        boolean isOutOfBounds = isOutOfBounds(closeBuffer, farBuffer);
        if (isOutOfBounds) {
            driveSystem.setPower(0);
            telemetry.log("driveTest", "distance buffer triggered");
        }
        while (isOutOfBounds(closeBuffer, farBuffer)) {
            telemetry.log("driveTest", "isOutOfBounds");

            /*boolean reachedStopCondition = false;
            switch (stopCondition) {
                case DISTANCE:
                    reachedStopCondition = (driveSystem.getMinDistanceFromTarget() < 50);
                    break;
                case DEPOT:
                    reachedStopCondition = isInDepot();
                    break;
                case CRATOR:
                    reachedStopCondition = isOnCrater();
                    break;
            }
            if (reachedStopCondition) {
                break;
            }*/

            double[] correctionPowers = getCorrectionTurnPower(closeBuffer, farBuffer, correctionPower);
            double leftPower = correctionPowers[0];
            double rightPower = correctionPowers[1];

            driveSystem.tankDrive(leftPower, rightPower);
            telemetry.log("LidarNavigationSystem", "leftPower: " + leftPower);
            telemetry.log("LidarNavigationSystem", "rightPower: " +  rightPower);
            telemetry.write();
        }
    }

    private boolean isOutOfBounds(double closeBuffer, double farBuffer) {
        return (getDistance1() >= farBuffer) ||
                (getDistance2() >= farBuffer) ||
                (getDistance1() <= closeBuffer) ||
                (getDistance2() <= closeBuffer);
    }

    public void driveAlongWallToCrater(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        driveSystem.setPower(adjustedPower);

        while (!isOnCrater()) {
            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.CRATOR);

            telemetry.log("LidarNavigationSystem","scaled power: " + adjustedPower);
            driveSystem.setPower(adjustedPower);
            telemetry.write();
        }
        driveSystem.setPower(0);
        telemetry.log("LidarNavigationSystem","reached crator");
        telemetry.write();
    }

    public void driveAlongWallToDepot(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        driveSystem.setPower(adjustedPower);

        while (!isInDepot()) {
            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.CRATOR);

            telemetry.log("LidarNavigationSystem","scaled power: " + adjustedPower);
            driveSystem.setPower(adjustedPower);
            telemetry.write();
        }
        driveSystem.setPower(0);
        telemetry.log("LidarNavigationSystem","reached depot");
        telemetry.write();
    }

    public void getCloseToWall(double targetDistanceFromWall, double power) {
        driveSystem.setPower(power);

        while ((getDistance2() > targetDistanceFromWall) && (getDistance1() > targetDistanceFromWall)) {
            driveSystem.setPower(power);
        }
        driveSystem.setPower(0);
    }

    public double[] getCorrectionTurnPower(double closeBuffer, double farBuffer, double power) {
        double turnPower = (power / 2);
        double rightPower = turnPower;
        double leftPower = turnPower;
        if ((((getDistance1() >= farBuffer) || getDistance2() <= closeBuffer) && !(getDistance2() >= farBuffer)) ||
                ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer))) {
            if (power > 0) {
                telemetry.log("driveTest", "turning RIGHT");
                rightPower = 0;
                leftPower = turnPower;
            } else {
                telemetry.log("driveTest", "turning LEFT");
                rightPower = turnPower;
                leftPower = 0;
            }
        } else if ((((getDistance2() >= farBuffer) || (getDistance1() <= closeBuffer))) ||
                ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer))) {
            if (power > 0) {
                telemetry.log("driveTest", "turning LEFT");
                rightPower = turnPower;
                leftPower = 0;
            } else {
                telemetry.log("driveTest", "turning RIGHT");
                rightPower = 0;
                leftPower = turnPower;
            }
        }
        double[] powers = new double[] {leftPower, rightPower};
        return powers;
    }

    public boolean isOnCrater() {
        double criticalAngle = 1.5;
        return ((Math.abs(imu.getPitch() - initPitch) < criticalAngle) ||
                (Math.abs(imu.getRoll() - initRoll) < criticalAngle));
    }

    public boolean isInDepot() {
        int redTriggerValue = 12;
        int blueTriggerValue = 8;
        return ((colorSystem.getRed() < redTriggerValue) ||
                (colorSystem.getBlue() < blueTriggerValue));
    }
}
