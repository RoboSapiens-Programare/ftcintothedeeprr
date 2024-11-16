package org.firstinspires.ftc.teamcode.robot;


import static org.firstinspires.ftc.teamcode.robot.UniversalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.UniversalValues.OUTTAKE_EXTEND_MID;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FSM DRIVE MODE", group = "FSMTELEOP")
public class fsmDriveMode extends OpMode {
    private Robot robot = null;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean isHorizontal = true;
    private boolean isStarted = true;
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
                if(intakeTimer.seconds() > 0.8){
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
                if(gamepad1.dpad_right){
                    robot.intake.setClawPivot(UniversalValues.CLAW_VERTICAL);
                    isHorizontal = false;
                }
                if(gamepad1.dpad_left){
                    robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                    isHorizontal = true;
                }
                if(gamepad1.left_trigger > 0.1){
                    robot.outtake.OpenOuttake(UniversalValues.OUTTAKE_OPEN);
                    robot.intake.setClawPivot(UniversalValues.CLAW_HORIZONTAL);
                    robot.intake.setPivot(UniversalValues.INTAKE_INT);
                    robot.intake.ManualLevel(INTAKE_LOW, 1);
                    intakeState = IntakeState.INTAKE_RETRACT;
                    intakeTimer.reset();
                }
                break;

            case INTAKE_RETRACT:
                if(robot.intake.intakeLimit.isPressed()){
                    robot.intake.setPivot(UniversalValues.INTAKE_UP);
                    if(intakeTimer.seconds() > UniversalValues.CLAW_TIMER){
                        robot.intake.OpenIntake(UniversalValues.CLAW_OPEN);
                        intakeTimer.reset();
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