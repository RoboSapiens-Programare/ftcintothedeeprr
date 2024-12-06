package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.UniversalValues;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Auto Bucket", group = "Examples")
public class ExampleBucketAuto extends OpMode {
    private Robot robot = null;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, actionTimer2;
    private boolean singleton = true;
    private boolean singleton2 = true;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */
    private final Pose startPose = new Pose(8,103, Math.toRadians(-90));
    private final Pose scorePose = new Pose(6,130, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(10, 125, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(24, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 96, Math.toRadians(90));
    private final Pose parkPose = new Pose(59,86, Math.toRadians(-90));
    private final Pose parkEmergencyPose = new Pose(14,29, Math.toRadians(-90));
    private boolean isPressed = false;

    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park, scorePreload;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        //score preload path



    scorePreload = follower.pathBuilder()
            .addPath(new BezierCurve(new Point(startPose),new Point(28.935, 119.103, Point.CARTESIAN), new Point(scorePose)))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
            .build();


        //grab pickup 1 path
    grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
            .build();

        //score pickup 1 path
    scorePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())


            .build();

        //grab pickup 2 path
    grabPickup2 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
            .build();

        //score pickup 2 path
    scorePickup2 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
            .build();

        //grab pickup 3 path
    grabPickup3 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
            .build();

        //score pickup 3 path
    scorePickup3 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
            .build();

        //park path
    park = follower.pathBuilder()
            .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
            .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
            .build();
    }



    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:

                //GOES TO SCORE POSITION
                robot.intake.setPivot(UniversalValues.INTAKE_INT);
                follower.followPath(scorePreload,true);
                actionTimer.resetTimer();
                setPathState(1);
                break;

            case 1:

                //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES PRELOAD, GOES TO GRABBING FIRST SAMPLE POSITION AND HOLDS
                if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                    if (actionTimer.getElapsedTimeSeconds() > 3) {
                            robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND, 1);
                        if (robot.outtake.outtakeMotor.getCurrentPosition() > UniversalValues.OUTTAKE_EXTEND - 50) {
                            if (singleton)
                            {
                                robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                                singleton = false;
                            }

                            if (actionTimer.getElapsedTimeSeconds() > 5) {
                                robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                                if(actionTimer.getElapsedTimeSeconds() > 6){
                                    if (singleton2) {
                                        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                                        singleton2 = false;
                                    }
                                }
                                if (actionTimer.getElapsedTimeSeconds() > 7) {
                                    robot.outtake.ManualLevel(0, 0.75);
                                    robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_OPEN);
                                    actionTimer.resetTimer();
                                    robot.intake.setPivot(UniversalValues.INTAKE_DOWN);
                                    follower.followPath(grabPickup1, /* holdEnd = */ true);
                                    singleton2 = true;
                                    singleton = true;
                                    setPathState(2);
                                }
                            }
                        }
                    }
                }
                break;

            case 2:

                //CHECKS IF ROBOT IS AT GRAB POSITION, GRABS SAMPLE, GOES TO SCORING POSITION AND HOLDS
                if(follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    if(actionTimer.getElapsedTimeSeconds() > 5) {
                        if (singleton) {
                            robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                            singleton = false;
                        }
                    }
                    if(actionTimer.getElapsedTimeSeconds() > 7) {
                        robot.intake.ManualLevel(0, 1);
                        robot.intake.setPivot(UniversalValues.INTAKE_UP);
                        if (robot.intake.intakeLimit.isPressed() || isPressed) {
                            isPressed = true;
                            if (singleton2) {
                                robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                                singleton2 = false;
                            }
                            if (actionTimer.getElapsedTimeSeconds() > 9) {
                                robot.intake.setPivot(UniversalValues.INTAKE_INT);
                                robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND, 1);
                                robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                                follower.followPath(scorePickup1, true);
                                actionTimer.resetTimer();
                                robot.intake.ManualLevel(0, 1);
                                isPressed = false;
                                singleton2 = true;
                                singleton = true;
                                setPathState(3);
                            }
                        }
                    }

                    else{
                        robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND, 1);
                        if (singleton2 && singleton){
                            robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                        }

                    }
                }
                break;
            case 3:

                //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES SAMPLE, GOES TO GRABBING SECOND SAMPLE POSITION AND HOLDS
                if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                    if(actionTimer.getElapsedTimeSeconds() > 3){
                        robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                        if(actionTimer.getElapsedTimeSeconds() > 5) {
                            robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                            robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                            robot.outtake.ManualLevel(0, 1);
                            actionTimer.resetTimer();
                            robot.intake.setPivot(UniversalValues.INTAKE_DOWN);
                            follower.followPath(grabPickup2, /* holdEnd = */ true);
                            singleton2 = true;
                            singleton = true;
                            setPathState(4);
                        }
                    }
                    else{
                        robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND, 1);
                        robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                    }
                }
                break;

            case 4:

                //CHECKS IF ROBOT IS AT GRAB POSITION, GRABS SAMPLE, GOES TO SCORING POSITION AND HOLDS
                if(follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
                    if(actionTimer.getElapsedTimeSeconds() > 3) {
                        if (singleton) {
                            robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                            singleton = false;
                        }
                        robot.intake.ManualLevel(0,1);
                        robot.intake.setPivot(UniversalValues.INTAKE_UP);
                        if(robot.intake.intakeLimit.isPressed() || isPressed){
                            isPressed = true;
                            if (singleton2) {
                                robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                                singleton2 = false;
                            }
                            if(actionTimer.getElapsedTimeSeconds() > 7) {
                                robot.intake.setPivot(UniversalValues.INTAKE_INT);
                                robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND_MID,1);
                                follower.followPath(scorePickup2,true);
                                actionTimer.resetTimer();
                                singleton2 = true;
                                singleton = true;
                                setPathState(5);
                                isPressed = false;
                            }
                        }
                    }
                    else{
                        if (singleton2 && singleton){
                            robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                        }
                        robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND, 1);
                    }
                }
                break;

            case 5:

                //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES SAMPLE, GOES TO GRABBING THIRD SAMPLE POSITION AND HOLDS
                if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                    if(actionTimer.getElapsedTimeSeconds() > 3){
                        robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                        if(actionTimer.getElapsedTimeSeconds() > 5) {
                            robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                            robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                            robot.outtake.ManualLevel(0, 1);
                            actionTimer.resetTimer();
                            robot.intake.setPivot(UniversalValues.INTAKE_DOWN);
                            follower.followPath(grabPickup3, /* holdEnd = */ true);
                            singleton2 = true;
                            singleton = true;
                            setPathState(6);
                        }
                    }
                    else{
                        robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND, 1);
                        robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                    }
                }
                break;

            case 6:

                //CHECKS IF ROBOT IS AT GRAB POSITION, GRABS SAMPLE, GOES TO SCORING POSITION AND HOLDS
                if(follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {
                    if(actionTimer.getElapsedTimeSeconds() > 3) {
                        if (singleton) {
                            robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                            singleton = false;
                        }
                        robot.intake.ManualLevel(0,1);
                        robot.intake.setPivot(UniversalValues.INTAKE_UP);
                        if(robot.intake.intakeLimit.isPressed() || isPressed){
                            isPressed = true;
                            if (singleton2) {
                                robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                                singleton2 = false;
                            }
                            if(actionTimer.getElapsedTimeSeconds() > 7) {
                                robot.intake.setPivot(UniversalValues.INTAKE_INT);
                                robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND_MID,1);
                                follower.followPath(scorePickup3,true);
                                actionTimer.resetTimer();
                                singleton2 = true;
                                singleton = true;
                                setPathState(7);
                                isPressed = false;
                            }
                        }
                    }
                    else{
                        if (singleton2 && singleton){
                            robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                        }
                        robot.intake.ManualLevel(UniversalValues.INTAKE_EXTEND, 1);
                    }
                }
                break;

            case 7:

                //CHECKS IF ROBOT IS AT SCORE POSITION, SCORES SAMPLE, GOES TO PARKING POSITION AND HOLDS
                if((follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1))) {
                    if(actionTimer.getElapsedTimeSeconds() > 3){
                        robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                        if(actionTimer.getElapsedTimeSeconds() > 5) {
                            robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                            robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                            robot.outtake.ManualLevel(0, 1);
                            actionTimer.resetTimer();
                            follower.followPath(park, /* holdEnd = */ true);
                            singleton2 = true;
                            singleton = true;
                            setPathState(8);
                        }
                    }
                    else{
                        robot.outtake.ManualLevel(UniversalValues.OUTTAKE_EXTEND, 1);
                        robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                    }
                }
                break;

            //CHECKS IF ROBOT IS AT PARKING POSITION, EXTEND SLIDES AT LEVEL 1 ASCENT POSITION
            case 8:
                if(follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    robot.intake.setPivot(UniversalValues.INTAKE_INIT);
                    robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                    singleton2 = true;
                    singleton = true;
                    setPathState(-1);
                }
                break;

            //IF ANYTHING GOES WRONG THEN EMERGENCY PARK
            case 9:
                robot.intake.ManualLevel(0,1);
                robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                robot.intake.setPivot(UniversalValues.INTAKE_INIT);
                robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                PathChain parkEmergency;
                parkEmergency = follower.pathBuilder()
                        .setZeroPowerAccelerationMultiplier(8)
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(parkEmergencyPose)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkEmergencyPose.getHeading())
                        .build();
                follower.followPath(parkEmergency, /* holdEnd = */ true);
                break;
        }

        }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer2 = new Timer();
        actionTimer2.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        telemetry.update();

        buildPaths();


        // Set the subsystems to positions for init
        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(UniversalValues.INTAKE_INIT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        actionTimer2.resetTimer();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
