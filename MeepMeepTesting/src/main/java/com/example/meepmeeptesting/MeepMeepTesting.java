package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // offset syntax:
        // x y and heading followed by off/offset, alliance color, position, and axis (nx for negative x)

        double xoff_bluemiddle_nx = -10;
        double yoff_bluemiddle_nx = 61.5;
        double headingoff_bluemiddle_nx = -1.5745;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 140, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0+xoff_bluemiddle_nx, 0+yoff_bluemiddle_nx, 0+headingoff_bluemiddle_nx))

                        .strafeRight(37)
                        .forward(10)
                        .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(xoff_bluemiddle_nx-48, yoff_bluemiddle_nx-10,  Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(-40, 40,  Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-40, 25,  Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(-40, 40,  Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                        .lineToLinearHeading(new Pose2d(47, 52, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))


                        .build());

        RoadRunnerBotEntity PushBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 140, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0+xoff_bluemiddle_nx, 0+yoff_bluemiddle_nx, 0+headingoff_bluemiddle_nx))


                        .strafeRight(25)
                        .forward(35)
                        .strafeRight(15)
                        .forward(15)
                        .strafeRight(10)
                        .back(40)
                        .lineToLinearHeading(new Pose2d(47, 52, Math.toRadians(-90)))
                        .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(PushBot)
                .start();
    }
}