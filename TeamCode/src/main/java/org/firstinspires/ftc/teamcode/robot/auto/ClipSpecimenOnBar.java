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
    private Timer stateTimer, pathTimer;
    private boolean singleton = true;
    private int pathState;

    private final Pose startPose = new Pose(10.7,49, Math.toRadians(0));
    private final Pose barCliponPose = new Pose(37.5,69.5, Math.toRadians(180));
    private PathChain toBar1;

    public void buildPaths() {
        // TODO : fix Linear Heading Interpolation for rotation to the right without hard coding value bigger than 180 (line 37)
        toBar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(barCliponPose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180.001))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        singleton = true;
        pathTimer.resetTimer();
        stateTimer.resetTimer();
    }

    public void autonomousUpdate() {
        switch(pathState) {
            case(0):
                robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                // TODO : fix intake pivot value on line 53 or make dedicated value in universalValues
                robot.intake.setPivot(0.55);
                follower.followPath(toBar1,true);
                setPathState(1);
            case(1):
                if((follower.getPose().getX() > (barCliponPose.getX() - 1) && follower.getPose().getY() > (barCliponPose.getY() - 1))) {
                    if (stateTimer.getElapsedTimeSeconds() > 3)
                    {
                        if (singleton){
                            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_UP);
                            singleton = false;
                        }
                        if (stateTimer.getElapsedTimeSeconds() > 5) {
                            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_DOWN);
                            if (stateTimer.getElapsedTimeSeconds() > 6)
                            {
                                setPathState(-1);
                            }
                        }
                    }
                }
        }
    }

    @Override
    public void init() {

        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);

        stateTimer = new Timer();
        pathTimer = new Timer();

        follower.setStartingPose(startPose);

        robot.intake.setPivot(universalValues.INTAKE_INIT);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);

        telemetry.update();
        buildPaths();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    @Override
    public void init_loop() {
        stateTimer.resetTimer();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
