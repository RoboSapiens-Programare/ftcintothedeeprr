package org.firstinspires.ftc.teamcode.robot.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.util.Timer;
import org.firstinspires.ftc.teamcode.robot.subsystems.robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.universalValues;

import java.sql.Time;

@Autonomous(name = "Specimen on bar", group = "Autonomous")
public class ClipSpecimenOnBar extends OpMode {

    private org.firstinspires.ftc.teamcode.robot.subsystems.robot robot = null;
    private Follower follower;
    private Timer underbartimer, clipontimer;
    private boolean singleton = true;

    public void specimenonbar()
    {

        if (gamepad1.cross)
        {
            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_UP);
        }

        if (gamepad1.square)
        {
            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_DOWN);
        }

        if (gamepad1.circle)
        {
            robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        }

        if (gamepad1.left_bumper)
        {
            robot.outtake.OpenOuttake(universalValues.OUTTAKE_CLOSE);
        }

        if (gamepad1.right_bumper)
        {
            robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
        }

    }

    private final Pose startPose = new Pose(10.7,49, Math.toRadians(0));
    private final Pose barCliponPose = new Pose(37.5,69.5, Math.toRadians(0));
    private PathChain toBar1;

    public void buildPaths() {
        toBar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(barCliponPose)))
                .build();
    }

    @Override
    public void init() {
        clipontimer = new Timer();
        underbartimer = new Timer();
        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.update();
        buildPaths();
    }

    @Override
    public void loop() {
            specimenonbar();
    }

    @Override
    public void init_loop() {
        underbartimer.resetTimer();
        clipontimer.resetTimer();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
    }
}
