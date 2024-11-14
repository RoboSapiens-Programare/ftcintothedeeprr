package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
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

            if(gamepad1.cross){
                robot.intake.setPivot(Values.INTAKE_UP);
            }
            if(gamepad1.circle){
                robot.intake.setPivot(Values.INTAKE_DOWN);
            }
            if(gamepad1.triangle){
                robot.outtake.setPivot(Values.OUTTAKE_UP);
            }
            if(gamepad1.square){
                robot.outtake.setPivot(Values.OUTTAKE_DOWN);
            }
            if(gamepad1.dpad_down){
                robot.intake.CloseIntake(Values.CLAW_CLOSE);
            }
            if(gamepad1.dpad_right){
                robot.intake.CloseIntake(Values.CLAW_OPEN);
            }
            if(gamepad1.dpad_left){
                robot.outtake.CloseOuttake(Values.OUTTAKE_CLOSE);
            }
            if(gamepad1.dpad_up){
                robot.outtake.OpenOuttake(Values.OUTTAKE_OPEN);
            }
            if(gamepad1.right_bumper){
                robot.intake.setClawPivot(Values.CLAW_HORIZONTAL);
            }
            if(gamepad1.left_bumper) {
                robot.intake.setClawPivot(Values.CLAW_VERTICAL);
            }

            if(gamepad1.right_trigger > 0.1){
                robot.intake.ManualLevel(manualTarget, 1);
                manualTarget += 3;
            }
            if(gamepad1.left_trigger > 0){
                if(!robot.intake.intakeLimit.isPressed()) {
                    robot.intake.ManualLevel(manualTarget, -1);
                    manualTarget -= 3;
                }
                else robot.intake.intakeMotor.setPower(0);
            }
            telemetry.addData("Touch Sensor State: ", robot.intake.intakeLimit.isPressed());
            telemetry.addData("Manual target: ", robot.intake.intakeMotor.getCurrentPosition());
            telemetry.update();
        }


    }
}
