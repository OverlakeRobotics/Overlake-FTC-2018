package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.controller.Handler;
import org.firstinspires.ftc.teamcode.systems.arm.ArmState;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.slide.SlideState;

@TeleOp(name="Slow Falling Test")
public class SlowFallingTestOpMode extends OpMode
{
    private MecanumDriveSystem driveSystem;
    private Controller controller;
    private ArmSystem arm;

    @Override
    public void init()
    {
        controller = new Controller(gamepad1);
        driveSystem = new MecanumDriveSystem(this);
        arm = new ArmSystem(this);
    }

    @Override
    public void loop()
    {
        driveSystem.mecanumDrive(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.left_stick_y, false);
        controller.handle();
    }
}
