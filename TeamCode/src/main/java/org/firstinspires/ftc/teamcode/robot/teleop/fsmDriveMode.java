package org.firstinspires.ftc.teamcode.robot.teleop;


import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.CLAW_HORIZONTAL;
import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.CLAW_VERTICAL;
import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.INTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.OUTTAKE_EXTEND;
import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.OUTTAKE_EXTEND_MID;
import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.PIVOT_TIMER;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystems.robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.universalValues;

@TeleOp(name = "FSM DRIVE MODE", group = "FSMTELEOP")
public class fsmDriveMode extends OpMode {
    private org.firstinspires.ftc.teamcode.robot.subsystems.robot robot = null;
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
        robot = new robot(hardwareMap);
        intakeTimer.reset();
        outtakeTimer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.intake.ManualLevel(0,1);
        robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
        robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
        robot.intake.setPivot(universalValues.INTAKE_INIT);
        robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
    }

    public void loop(){
        //telemetry.addData("Claw Pivot", clawPivot);
        telemetry.update();
//       telemetry.addData("cm: ", robot.outtake.outtakeSensor.getDistance(DistanceUnit.CM));
        switch(intakeState){
            case INTAKE_START:
                isPressed = false;
                if(isStarted){
                    robot.intake.setPivot(universalValues.INTAKE_INT);
                    isStarted = false;
                }
                if(gamepad1.right_trigger > 0.1){
                    robot.intake.ManualLevel(INTAKE_EXTEND, 1);
                    intakeState = IntakeState.INTAKE_EXTEND;
                }
                if(intakeTimer.seconds() > 0.2){
                    robot.intake.setPivot(universalValues.INTAKE_INT);
                    robot.outtake.CloseOuttake(universalValues.OUTTAKE_CLOSE);
                }
                if(gamepad2.triangle){
                    robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID,1);
                    intakeState = IntakeState.OUTTAKE_MID;
                }
                if(gamepad1.dpad_down){
                    robot.outtake.setPivot(universalValues.OUTTAKE_DUMP);
                }
                break;

            case INTAKE_EXTEND:
                if(Math.abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 10){
                    robot.intake.setPivot(universalValues.INTAKE_DOWN);
                    robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    robot.intake.CloseIntake(universalValues.CLAW_OPEN);
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
                }
                break;

            case INTAKE_CLAW_COLLECT_POSITION:
                if(gamepad1.left_bumper){
                    robot.intake.CloseIntake(universalValues.CLAW_CLOSE);
                    robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                }
                if(gamepad1.right_bumper){
                    robot.intake.OpenIntake(universalValues.CLAW_OPEN);
                    robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
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
                    robot.intake.setClawPivot(universalValues.CLAW_VERTICAL);
                    clawPivot = CLAW_VERTICAL;
                    isHorizontal = false;
                }
                if(gamepad1.dpad_up){
                    robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    isHorizontal = true;
                }



                if(gamepad1.left_trigger > 0.1){
                    robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
                    robot.intake.setClawPivot(universalValues.CLAW_HORIZONTAL);
                    clawPivot = CLAW_HORIZONTAL;
                    robot.intake.setPivot(universalValues.INTAKE_INT);
                    robot.intake.ManualLevel(INTAKE_LOW, 1);
                    intakeState = IntakeState.INTAKE_RETRACT;
                    intakeTimer.reset();
                }
                break;

            case INTAKE_RETRACT:
                if(robot.intake.intakeLimit.isPressed()) isPressed = true;
                if(isPressed) {
                    robot.intake.setPivot(universalValues.INTAKE_UP);

                    if (intakeTimer.seconds()> universalValues.CLAW_TIMER) {
                        robot.intake.OpenIntake(universalValues.CLAW_OPEN);
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
                robot.outtake.setPivot(universalValues.OUTTAKE_DUMP);
                if(gamepad2.right_bumper){
                    robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
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
                robot.outtake.setPivot(universalValues.OUTTAKE_DUMP);
                if(gamepad2.right_bumper){
                    robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
                }
                break;

            case OUTTAKE_RETRACT:
                robot.outtake.setPivot(universalValues.OUTTAKE_COLLECT);
                robot.outtake.OpenOuttake(universalValues.OUTTAKE_OPEN);
                intakeState = IntakeState.INTAKE_START;
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }



        robot.drive.setDrivePower(new Pose2d((-gamepad1.left_stick_y) * 0.8,(-gamepad1.left_stick_x) * 0.8,(-gamepad1.right_stick_x) * 0.8));



    }

}