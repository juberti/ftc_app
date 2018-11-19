package org.firstinspires.ftc.teamcode.RoverRuckus.Deployers.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoverRuckus.Auto.SophisticatedSampling;

@Autonomous(name="Depo double sampling")
public class DepoDoubleSample extends SophisticatedSampling {
    @Override
    public void runOpMode() throws InterruptedException {
        this.goal = EndGoal.BLUE_DOUBLE_SAMPLE;
        this.startingPosition = StartingPosition.DEPOT;
        super.runOpMode();
    }
}
