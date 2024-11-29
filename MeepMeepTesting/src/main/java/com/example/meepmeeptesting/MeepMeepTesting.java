package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double xoffset = -10;
        double yoffset = 61.5;
        double headingoffset = -1.5745;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 140, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0+xoffset, 0+yoffset, 0+headingoffset))



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

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}