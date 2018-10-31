package org.firstinspires.ftc.teamcode.RoverRuckus.Deployers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoverRuckus.BaseTeleOp;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.SoloMapping;
import org.firstinspires.ftc.teamcode.RoverRuckus.Mappings.TandemMapping;
@TeleOp(name="Sparky Tandem")
public class TandemTeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() {
        this.controller = new TandemMapping(gamepad1, gamepad2);
        super.runOpMode();
    }
}