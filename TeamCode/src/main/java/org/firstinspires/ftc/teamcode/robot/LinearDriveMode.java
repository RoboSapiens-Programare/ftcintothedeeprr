package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    private UniversalValues Values = new UniversalValues();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }
        //INIT CODE

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                robot.intake.CloseIntake(Values.CLAW_CLOSE);
                robot.outtake.setPivot(Values.OUTTAKE_DOWN);
                robot.intake.setPivot(Values.INTAKE_INT);
                sleep(50);
                robot.intake.setPivot(Values.INTAKE_UP);
                sleep(2000);
                robot.intake.CloseIntake(Values.CLAW_OPEN);
                sleep(500);
                robot.intake.setPivot(Values.INTAKE_DOWN);
            }
        }


    }
}
