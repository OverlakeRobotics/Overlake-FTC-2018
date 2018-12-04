package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.controller.Controller;
import org.firstinspires.ftc.teamcode.hardware.controller.Handler;
import org.firstinspires.ftc.teamcode.opmodes.debuggers.TeleOpModeDebugger;
import org.firstinspires.ftc.teamcode.systems.arm.ArmState;
import org.firstinspires.ftc.teamcode.systems.arm.ArmSystem;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.flail.Flail;
import org.firstinspires.ftc.teamcode.systems.slide.SlideState;
import org.firstinspires.ftc.teamcode.systems.slide.SlideSystem;

@TeleOp(name = "BuggedTeleop", group="TeleOp")
public class PushBotBoiz extends OpMode
{
    private Controller controller1;
    private MecanumDriveSystem driveSystem;

    private boolean slowDrive;

    public PushBotBoiz() {
        msStuckDetectLoop = 1000000000;
    }

    @Override
    public void init()
    {
        this.driveSystem = new MecanumDriveSystem(this);
        this.controller1 = new Controller(gamepad1);

        initButton();

        slowDrive = false;
    }

    @Override
    public void loop()
    {
        controller1.handle();

        float rx = controller1.gamepad.right_stick_x;
        float ry = controller1.gamepad.right_stick_y;
        float lx = controller1.gamepad.left_stick_x;
        float ly = controller1.gamepad.left_stick_y;

        driveSystem.mecanumDrive(rx, ry, lx, ly, slowDrive);

    }

    public void initButton() {
        telemetry.addData("buttons", "initialize");
        telemetry.update();

    }
}
