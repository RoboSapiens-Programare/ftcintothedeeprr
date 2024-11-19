package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.robot.UniversalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.PIVOT_TIMER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "FSM DRIVE MODE", group = "FSMTELEOP")
public class fsmDriveMode extends OpMode {
    private Robot robot = null;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean isHorizontal = true;
    private boolean isStarted = true;
    private boolean isPressed = false;
    public enum IntakeState {
        INTAKE_START,
        INTAKE_CLAW_COLLECT_POSITION,
        INTAKE_RETRACT,
        INTAKE_EXTEND,
        OUTTAKE_MID,
        OUTTAKE_EXTEND,
        OUTTAKE_RETRACT
    };

    final int OUTTAKE_LOW = 0;
    final int INTAKE_LOW = 0;

    double clawPivot = CLAW_HORIZONTAL;
    final double DUMP_TIME = 1.5;

    IntakeState intakeState = IntakeState.INTAKE_START;

    public void init() {
        robot = new Robot(hardwareMap);
        intakeTimer.reset();
        outtakeTimer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(UniversalValues.INTAKE_INIT);
        robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
    }

    public void loop(){
        //telemetry.addData("Claw Pivot", clawPivot);
        //telemetry.update();
        switch(intakeState){
            case INTAKE_START:
                if(isStarted){
                    robot.intake.setPivot(UniversalValues.INTAKE_INT);
                    isStarted = false;
                }
                if(gamepad1.right_trigger > 0.1){
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                if(intakeTimer.seconds() > 0.2){
                    robot.intake.setPivot(UniversalValues.INTAKE_INT);
                    robot.outtake.CloseOuttake(UniversalValues.OUTTAKE_CLOSE);
                }
                if(gamepad2.triangle){
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID,1);
                    intakeState = IntakeState.OUTTAKE_MID;
                }
                if(gamepad1.dpad_down){
                    robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                }
                break;

            case INTAKE_EXTEND:
                if(Math.abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 10){
                    robot.intake.setPivot(UniversalValues.INTAKE_DOWN);
                    robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    robot.intake.CloseIntake(UniversalValues.CLAW_OPEN);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
                }
                break;

            case INTAKE_CLAW_COLLECT_POSITION:
                if(gamepad1.left_bumper){
                    robot.intake.CloseIntake(UniversalValues.CLAW_CLOSE);
                    robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                }
                if(gamepad1.right_bumper){
                    robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                    robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                }

                if(gamepad1.dpad_right && outtakeTimer.seconds() > PIVOT_TIMER)
                {
                    if (clawPivot<1) {
                        clawPivot+=0.01;
                        robot.intake.setClawPivot(clawPivot);
                        outtakeTimer.reset();
                    }
                }

                if(gamepad1.dpad_left && outtakeTimer.seconds() > PIVOT_TIMER)
                {
                    if (clawPivot>CLAW_VERTICAL) {
                        clawPivot -= 0.01;
                        robot.intake.setClawPivot(clawPivot);
                        outtakeTimer.reset();
                    }
                }

                if(gamepad1.dpad_down){
                    robot.intake.setClawPivot(UniversalValues.CLAW_VERTICAL);
                    clawPivot = CLAW_VERTICAL;
                    isHorizontal = false;
                }
                if(gamepad1.dpad_up){
                    robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    isHorizontal = true;
                }



                if(gamepad1.left_trigger > 0.1){
                    robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                    robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    robot.intake.setPivot(UniversalValues.INTAKE_INT);
                    robot.intake.ManualLevel(INTAKE_LOW, 1);
                    intakeState = IntakeState.INTAKE_RETRACT;
                    intakeTimer.reset();
                }
                break;

            case INTAKE_RETRACT:

                //telemetry.update();
                if(robot.intake.intakeLimit.isPressed()) isPressed = true;
                if(isPressed) {
                    robot.intake.setPivot(UniversalValues.INTAKE_UP);
                    //telemetry.addData("cm: ", robot.outtake.outtakeSensor.getDistance(DistanceUnit.CM));
                    if (robot.outtake.outtakeSensor.getDistance(DistanceUnit.CM) < 14 && robot.outtake.outtakeSensor.getDistance(DistanceUnit.CM) > 0.1) {
                        robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                        intakeTimer.reset();
                        isPressed = false;
                        intakeState = IntakeState.INTAKE_START;
                    }
                }
                break;

            case OUTTAKE_MID:
                if(gamepad2.cross){
                    robot.outtake.ManualLevel(OUTTAKE_LOW, 0.4);
                    intakeState = IntakeState.OUTTAKE_RETRACT;
                }
                robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                if(gamepad2.right_bumper){
                    robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                }
                if(Math.abs(robot.outtake.outtakeMotor.getCurrentPosition() - OUTTAKE_EXTEND_MID) < 100){
                    if(gamepad2.triangle){
                        robot.outtake.ManualLevel(OUTTAKE_EXTEND, 1);
                        intakeState = IntakeState.OUTTAKE_EXTEND;
                    }
                }
                break;

            case OUTTAKE_EXTEND:
                if(gamepad2.cross){
                    robot.outtake.ManualLevel(OUTTAKE_LOW, 0.4);
                    intakeState = IntakeState.OUTTAKE_RETRACT;
                }
                robot.outtake.setPivot(UniversalValues.OUTTAKE_DUMP);
                if(gamepad2.right_bumper){
                    robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                }
                break;

            case OUTTAKE_RETRACT:
                robot.outtake.setPivot(UniversalValues.OUTTAKE_COLLECT);
                robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                intakeState = IntakeState.INTAKE_START;
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }



        robot.drive.setDrivePower(new Pose2d((-gamepad1.left_stick_y) * 0.8,(-gamepad1.left_stick_x) * 0.8,(-gamepad1.right_stick_x) * 0.8));



    }

}