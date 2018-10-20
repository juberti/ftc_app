package org.firstinspires.ftc.teamcode.RoverRuckusAuto;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveSystems.Mecanum.RoadRunner.RoadRunnerMecanumInterface;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name="splineFollow")
@Config
public class SplineFollowOpMode extends LinearOpMode {

    public static PIDCoefficients TRANSLATIONAL_COEFFS = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_COEFFS = new PIDCoefficients(0, 0, 0);
    private static double K_V = 0.014;
    private static double K_A = 0;
    private static double K_STATIC = 0.07096;
    private static double MAX_VELO = 40;
    private static double MAX_ACCEL = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket k = new TelemetryPacket();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        RoadRunnerMecanumInterface drive = new RoadRunnerMecanumInterface(hardwareMap);
        // change these constraints to something reasonable for your drive
        DriveConstraints baseConstraints = new DriveConstraints(MAX_VELO, MAX_ACCEL, Math.PI / 2, Math.PI/2);
        MecanumConstraints constraints = new MecanumConstraints(baseConstraints, drive.getTrackWidth(), drive.getWheelBase());
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(-13, 13, Math.PI*0.75), constraints)
                .beginComposite()
                .splineTo(new Pose2d(-48, 24, Math.PI*0.75))
                .splineTo(new Pose2d(-60, 60, Math.PI*.5))
                /*.splineTo(new Pose2d(-20, 20, Math.PI*1.75))
                .splineTo(new Pose2d(10, 10, Math.PI/4))
                .splineTo(new Pose2d(40, 40, Math.PI/4))*/
                .closeComposite()
                .build();

        // DONE: tune kV, kA, and kStatic in the following follower
        // then tune the PID coefficients after you verify the open loop response is roughly correct
        MecanumPIDVAFollower follower = new MecanumPIDVAFollower(
                drive,
                TRANSLATIONAL_COEFFS,
                HEADING_COEFFS,
                K_V,
                K_A,
                K_STATIC);

        FileWriter fW = null;
        try {
            fW = new FileWriter(getFile());
        } catch (IOException e) {
            e.printStackTrace();
        }

        waitForStart();

        follower.followTrajectory(trajectory);
        int i = 0;
        int j = 0;
        while (opModeIsActive() && follower.isFollowing()) {

            Pose2d currentPose = drive.getPoseEstimate();

            /*TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            fieldOverlay.setStroke("green");
            DashboardUtil.drawSampledTrajectory(fieldOverlay, trajectory);
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(currentPose.getX(), currentPose.getY(), 3);

            packet.put("frontLeft", drive.leftFront.getPower());
            packet.put("frontRight", drive.rightFront.getPower());
            packet.put("backLeft", drive.leftRear.getPower());
            packet.put("backRight", drive.rightRear.getPower());
            packet.put("currentX", currentPose.getX());
            packet.put("currentY", currentPose.getY());
            packet.put("currentHeading", currentPose.getHeading());
            dashboard.sendTelemetryPacket(packet);*/

            try {
                fW.write(drive.leftFront.getPower() + ",");
                fW.write(drive.rightFront.getPower() + ",");
                fW.write(drive.leftRear.getPower() + ",");
                fW.write(drive.rightRear.getPower() + ",");
                fW.write(currentPose.getX() + ",");
                fW.write(currentPose.getY() + ",");
                fW.write(currentPose.getHeading() + ",");
                fW.write("\n");
            } catch (IOException e) {telemetry.log().add("Crashed in writing");}


            follower.update(currentPose);
            i++;
            telemetry.addData("Update", i);
            telemetry.update();

            drive.updatePoseEstimate();
            j++;
            telemetry.addData("PoseEstimate", j);
            telemetry.update();

        }
        try {
            fW.close();
        } catch (IOException e) {telemetry.log().add("Crashed in closing");}
        telemetry.update();
    }
    public File getFile() {
        final File folder = new File(Environment.getExternalStorageDirectory().toString(), "FIRST/LOGS/");
        if (!folder.exists()) {folder.mkdirs();}

        long time = System.currentTimeMillis();
        String name = time + ".csv";
        return new File(folder, name);
    }
}
