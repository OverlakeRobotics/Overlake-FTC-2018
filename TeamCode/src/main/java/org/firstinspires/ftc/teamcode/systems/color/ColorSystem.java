package org.firstinspires.ftc.teamcode.systems.color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.System;



    public class ColorSystem extends System {

        private ColorSensor colorSensor;
        private IBaseOpMode opMode;

        public ColorSystem(IBaseOpMode opMode) {
            super(opMode, "ColorSystem");

            this.opMode = opMode;
            colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
            colorSensor.enableLed(true);
        }

        public void telemetry() {
            opMode.getTelemetry().addData("Alpha", "Alpha: " + colorSensor.alpha());
            opMode.getTelemetry().addData("Red  ", "Red: " + colorSensor.red());
            opMode.getTelemetry().addData("Green", "Green: " + colorSensor.green());
            opMode.getTelemetry().addData("Blue ", "Blue: " + colorSensor.blue());
            opMode.getTelemetry().addData("B/G", "Blue / Green" + (colorSensor.blue() / colorSensor.green()));
            opMode.getTelemetry().addData("R/G", "Red / Green" + (colorSensor.red() / colorSensor.green()));
            opMode.getTelemetry().update();
        }

        public int getRed() {
            return colorSensor.red();
        }

        public int getBlue() {
            return colorSensor.blue();
        }

        public int getGreen() {
            return colorSensor.green();
        }

        public int getAlpha() {
            return colorSensor.alpha();
        }
    }
