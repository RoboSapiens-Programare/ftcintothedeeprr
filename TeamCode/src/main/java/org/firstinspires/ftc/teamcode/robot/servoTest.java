package org.firstinspires.ftc.teamcode.robot;

import android.renderscript.Sampler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class servoTest extends LinearOpMode {
    private Robot robot = null;
    private UniversalValues Values = new UniversalValues();
    public int manualTarget = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();
        boolean outtakeOpen = false;

        robot = new Robot(hardwareMap);
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }
        //INIT CODE

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.triangle){
                robot.intake.setPivot(Values.INTAKE_UP);
                }
            if(gamepad1.square){
                robot.intake.setPivot(Values.INTAKE_DOWN);
            }
            telemetry.update();
        }


    }
}
