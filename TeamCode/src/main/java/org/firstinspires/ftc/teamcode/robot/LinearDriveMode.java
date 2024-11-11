package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    private UniversalValues values = new UniversalValues();

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
            if(gamepad1.cross){
                robot.intake.setPivot(values.INTAKE_UP);
            }
            if(gamepad1.circle){
                robot.intake.setPivot(values.INTAKE_DOWN);
            }
            if(gamepad1.triangle){
                robot.outtake.setPivot(values.OUTTAKE_UP);
            }
            if(gamepad1.square){
                robot.outtake.setPivot(values.OUTTAKE_DOWN);
            }
            if(gamepad1.dpad_down){
                robot.intake.CloseIntake(values.CLAW_CLOSE);
            }
            if(gamepad1.dpad_right){
                robot.intake.CloseIntake(values.CLAW_OPEN);
            }
            if(gamepad1.dpad_left){
                robot.outtake.CloseOuttake(values.OUTTAKE_CLOSE);
            }
            if(gamepad1.dpad_up){
                robot.outtake.OpenOuttake(values.OUTTAKE_OPEN);
            }
            if(gamepad1.right_bumper){
                robot.intake.setClawPivot(values.CLAW_HORIZONTAL);
            }
            if(gamepad1.left_bumper){
                robot.intake.setClawPivot(values.CLAW_VERTICAL);
            }
        }


    }
}
