package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.robot.subsystems.robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.universalValues;

@Autonomous(name = "Auto Park Specimen", group = "Autonomous")
public class parkSpecimen extends OpMode {
private org.firstinspires.ftc.teamcode.robot.subsystems.robot robot = null;
private Follower follower;
private int pathState;
private final Pose startPose = new Pose(8,60, Math.toRadians(0));
private final Pose parkPose = new Pose(8,10, Math.toRadians(0));

private PathChain park;
private boolean singleton = true;

public void buildPaths() {
park = follower.pathBuilder()
        .addPath(new BezierLine(new Point(startPose), new Point(parkPose)))
        .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
        .build();
}

public void autonomousPathUpdate() {
if (singleton) {
    robot.intake.setPivot(universalValues.INTAKE_INT);
    follower.followPath(park);
    singleton = false;
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

@Override
public void loop() {
    follower.update();
    autonomousPathUpdate();
}
}
