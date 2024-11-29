package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class MeepMeepTesting extends LinearOpMode {

    public static double DISTANCE = 20;

    double xoffset = -10;
    double yoffset = 61.5;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(37)
                .forward(10)
                .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(xoffset-48, yoffset-10,  Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(-40, 40,  Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40, 25,  Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-40, 40,  Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                .lineToLinearHeading(new Pose2d(47, 52, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                .build();



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectory1);
        }
    }
}