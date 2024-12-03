package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name = "Autonomie_ALBASTRU_APROAPE", group="autonomous")

public class MeepMeepTesting extends LinearOpMode {
    private Robot robot = null;




    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        robot = new Robot(hardwareMap);

        telemetry.addData("has initialised", "yes");
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();




        waitForStart();



        while (opModeIsActive()) {
            Pose2d start = new Pose2d(-10, 61.5, Math.toRadians(-90));
            robot.drive.setPoseEstimate(start);
            TrajectorySequence trajectory1 = robot.drive.trajectorySequenceBuilder(start)
                    .forward(5)
                    .lineToLinearHeading(new Pose2d(52, 52, Math.toRadians(-135)))
                    .build();

            robot.drive.followTrajectorySequence(trajectory1);
            sleep(100000);
        }


        //        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
