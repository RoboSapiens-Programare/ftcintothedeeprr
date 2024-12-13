package org.firstinspires.ftc.teamcode.robot.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.util.Timer;
import org.firstinspires.ftc.teamcode.robot.subsystems.robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.universalValues;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import dev.frozenmilk.dairy.core.util.controller.calculation.pid.UnitComponent;

@Autonomous(name = "Specimen on bar", group = "Autonomous")
public class ClipSpecimenOnBar extends OpMode {

    private org.firstinspires.ftc.teamcode.robot.subsystems.robot robot = null;
    private Follower follower;
    private Timer stateTimer, pathTimer;
    private boolean singleton = true;
    private boolean singleton2 = true;
    private int pathState;

    // TODO: modify y offset to correct pedro path visualiser offset

    private final Pose startPose = new Pose(10.7,49, Math.toRadians(180));
    private final Pose barCliponPose1 = new Pose(32.75,70.5, Math.toRadians(180));
    private final Pose behindSample1 = new Pose(65, 29.500, Math.toRadians(180));
    private final Pose pushSample1 = new Pose(18.272, 29.400, Math.toRadians(180));
    private final Pose behindSample2 = new Pose(64.760, 19.069, Math.toRadians(180));
    private final Pose pushSample2 = new Pose(18.272, 18.620, Math.toRadians(180));
    private final Pose behindSample3 = new Pose(64.535, 12.556, Math.toRadians(180));
    private final Pose pushSample3 = new Pose(18.272, 12.332, Math.toRadians(180));


    private PathChain toBar1, toSample1, toHuman1, toSample2, toHuman2, toSample3, toHuman3;

    public void buildPaths() {
        // TODO : fix Linear Heading Interpolation for rotation to the right without hard coding value bigger than 180 (line 37)
        toBar1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(barCliponPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSample1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(barCliponPose1),
                                new Point(29.420, 39.875, Point.CARTESIAN),
                                new Point(38.628, 34.485, Point.CARTESIAN),
                                new Point(61.535, 37.404, Point.CARTESIAN),
                                new Point(behindSample1)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toHuman1 = follower.pathBuilder()
                //.setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierLine(new Point(behindSample1), new Point(pushSample1)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSample2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pushSample1),
                                new Point(74.866, 32.544, Point.CARTESIAN),
                                new Point(behindSample2)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample2), new Point(pushSample2)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pushSample2),
                        new Point(74.866, 20.417, Point.CARTESIAN),
                        new Point(behindSample3)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        toHuman3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample3), new Point(pushSample3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                //robot.intake.setPivot(0.55);
                    follower.followPath(toBar1,true);

                setPathState(1);
                break;
            case(1):
                telemetry.addData("statetimer", stateTimer.getElapsedTimeSeconds());
                if((follower.getPose().getX() > (barCliponPose1.getX() - 1) && follower.getPose().getY() > (barCliponPose1.getY() - 1))) {
                    if (singleton) {
                        follower.holdPoint(barCliponPose1);
                        singleton = false;
                    }
                    if (stateTimer.getElapsedTimeSeconds() > 1)
                    {
                        if (singleton2){
                            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_UP);
                            robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
                            singleton2 = false;
                        }
                        if (stateTimer.getElapsedTimeSeconds() > 1.5) {
                            robot.outtake.setPivot(universalValues.OUTTAKE_CLIPON_DOWN);
//                            follower.setMaxPower(0.5);
//                            follower.followPath(cliponBar1, true);
                            if (stateTimer.getElapsedTimeSeconds() > 2)
                            {
                                robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN_BAR);
                                if (stateTimer.getElapsedTimeSeconds() > 2.25)
                                {
                                    follower.followPath(toSample1);
                                    robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                                    robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
                                    setPathState(2);
                                }

                            }
                        }
                    }
                }
                break;
            case(2):
                if((follower.getPose().getX() > (behindSample1.getX() - 1) && follower.getPose().getY() > (behindSample1.getY() + 1)))
                {
                    follower.followPath(toHuman1, true);
                    setPathState(3);
                }
                break;
            case(3):
                if((follower.getPose().getX() < (pushSample1.getX() + 1) && follower.getPose().getY() < (pushSample1.getY() + 1)))
                {
                    follower.followPath(toSample2, true);
                    setPathState(4);
                }
                break;
            case(4):
                if((follower.getPose().getX() > (behindSample2.getX() - 1) && follower.getPose().getY() > (behindSample2.getY() - 1)))
                {
                    follower.followPath(toHuman2, true);
                    setPathState(5);
                }
                break;
            case(5):
                if((follower.getPose().getX() < (pushSample2.getX() + 1) && follower.getPose().getY() < (pushSample2.getY() + 1)))
                {
                    follower.followPath(toSample3, true);
                    setPathState(6);
                }
                break;
            case(6):
                if((follower.getPose().getX() > (behindSample3.getX() - 1) && follower.getPose().getY() > (behindSample3.getY() - 1)))
                {
                    follower.followPath(toHuman3, true);
                    setPathState(7);
                }
                break;
        }
    }

    @Override
    public void init() {

        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);

        stateTimer = new Timer();
        pathTimer = new Timer();

        follower.setStartingPose(startPose);

        //robot.intake.setPivot(universalValues.INTAKE_DOWN);
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
    public void     stop() {
    }
}
