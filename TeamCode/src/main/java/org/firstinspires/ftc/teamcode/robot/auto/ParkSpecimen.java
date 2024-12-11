package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.subsystems.robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.universalValues;

@Autonomous(name = "Auto Push Specimen", group = "Autonomous")
public class ParkSpecimen extends OpMode {
    private org.firstinspires.ftc.teamcode.robot.subsystems.robot robot = null;
    private Follower follower;

    private enum PathState {
        PATH_START, SCORE_FIRST_SAMPLE, GRAB_SECOND_SAMPLE, SCORE_SECOND_SAMPLE,
        GRAB_THIRD_SAMPLE, SCORE_THIRD_SAMPLE,
        PATH_END
    }
    private PathState pathState;
    private final Pose startPose = new Pose(8,63, Math.toRadians(0)); // visualizer: (x: 10 y: 46)
    private final Pose firstSampleGrabPose = new Pose(70, 28, Math.toRadians(0));
    private final Pose firstSampleScorePose = new Pose(8, 28, Math.toRadians(0));
    private final Pose secondSampleGrabPose = new Pose(70, 17, Math.toRadians(0));
    private final Pose secondSampleScorePose = new Pose(8, 17, Math.toRadians(0));
    private final Pose thirdSampleGrabPose = new Pose(70, 12, Math.toRadians(0));
    private final Pose thirdSampleScorePose = new Pose(8, 12, Math.toRadians(0));
    // private final Pose
    private boolean singleton = true;
    private PathChain firstSampleGrab, secondSampleGrab, thirdSampleGrab, firstSampleScore, secondSampleScore, thirdSampleScore;


    private void buildPaths() {
        firstSampleGrab = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(16.168, 24.757, Point.CARTESIAN),
                                new Point(45.103, 50.551, Point.CARTESIAN),
                                new Point(firstSampleGrabPose)
                        )
                ).build();
        firstSampleScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(firstSampleGrabPose),
                                new Point(firstSampleScorePose)
                        )
                )
                .build();

        secondSampleGrab = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(firstSampleScorePose),
                                new Point(61.925, 32.159, Point.CARTESIAN),
                                new Point(secondSampleGrabPose)
                        )
                )
                .build();
        secondSampleScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(secondSampleGrabPose),
                                new Point(secondSampleScorePose)
                        )
                )
                .build();

        thirdSampleGrab = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(secondSampleScorePose),
                                new Point(75.832, 22.065, Point.CARTESIAN),
                                new Point(thirdSampleGrabPose)
                        )
                )
                .build();
        thirdSampleScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(thirdSampleGrabPose),
                                new Point(thirdSampleScorePose)
                        )
                )
                .build();
    }

    private void setPathState(PathState state) {
        pathState = state;
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case PATH_START:
                follower.followPath(firstSampleGrab, true);
                setPathState(PathState.SCORE_FIRST_SAMPLE);
                break;
            case SCORE_FIRST_SAMPLE:
                follower.followPath(firstSampleScore);
                setPathState(PathState.GRAB_SECOND_SAMPLE);
                break;
            case GRAB_SECOND_SAMPLE:
                follower.followPath(secondSampleGrab);
                setPathState(PathState.SCORE_SECOND_SAMPLE);
                break;
            case SCORE_SECOND_SAMPLE:
                follower.followPath(secondSampleScore);
                setPathState(PathState.GRAB_THIRD_SAMPLE);
                break;
            case GRAB_THIRD_SAMPLE:
                follower.followPath(thirdSampleGrab);
                setPathState(PathState.SCORE_THIRD_SAMPLE);
                break;
            case SCORE_THIRD_SAMPLE:
                follower.followPath(thirdSampleScore);
                setPathState(PathState.PATH_END);
                break;
            case PATH_END:
                // maybe park robot somewhere else
                break;
        }

    }


    @Override
    public void init() {
        robot = new robot(hardwareMap);


        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.update();

        buildPaths();


        // Set the subsystems to positions for init
        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INIT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
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
}