package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.MecanumHardware;
import org.firstinspires.ftc.teamcode.Utilities.Control.StickyGamepad;

public class MainTeleOp extends LinearOpMode {

    final static double HEADING_INTERVAL = Math.PI / 4;

    MecanumHardware robot;
    StickyGamepad gp1, gp2;
    boolean slowMode;

    @Override
    public void runOpMode() {
        robot = new MecanumHardware(this);
        robot.init();
        slowMode = false;

        waitForStart();

        while (opModeIsActive()) {
            double[] unscaledMotorPowers = robot.getDrivePowersFromAngle(getControllerDir());

        }
    }

    public double getControllerDir() {
        double controllerAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI / 2;
        return Math.round(controllerAngle / HEADING_INTERVAL) * HEADING_INTERVAL;
    }


}
