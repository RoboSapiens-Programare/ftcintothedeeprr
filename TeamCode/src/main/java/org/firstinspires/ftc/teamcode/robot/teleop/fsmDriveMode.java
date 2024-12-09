package org.firstinspires.ftc.teamcode.robot.teleop;

import static org.firstinspires.ftc.teamcode.robot.subsystems.universalValues.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.follower.Follower;
import org.firstinspires.ftc.teamcode.robot.subsystems.drive.localization.Pose;
import org.firstinspires.ftc.teamcode.robot.subsystems.robot;

@TeleOp(name = "FSM DRIVE MODE", group = "FSMTELEOP")
public class fsmDriveMode extends OpMode {
    private robot robot;
    private Follower follower;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime outtakeTimer = new ElapsedTime();
    private boolean isHorizontal = true;
    private boolean isStarted = true;
    private boolean isPressed = false;
    
    private enum IntakeState {
        INTAKE_START, INTAKE_CLAW_COLLECT_POSITION, INTAKE_RETRACT, INTAKE_EXTEND,
        OUTTAKE_MID, OUTTAKE_EXTEND, OUTTAKE_RETRACT
    }
    
    private static final int OUTTAKE_LOW = 0;
    private static final int INTAKE_LOW = 0;

    private double clawPivot = CLAW_HORIZONTAL;
    private IntakeState intakeState = IntakeState.INTAKE_START;

    private void initializeRobot() {
        robot.intake.ManualLevel(0, 1);
        robot.intake.CloseIntake(CLAW_CLOSE);
        robot.intake.setClawPivot(CLAW_HORIZONTAL);
        robot.intake.setPivot(INTAKE_INIT);
        robot.outtake.setPivot(OUTTAKE_COLLECT);
        robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
    }

    private void handleIntakeStart() {
        isPressed = false;
        if (isStarted) {
            robot.intake.setPivot(INTAKE_INT);
            isStarted = false;
        }
        if (gamepad1.right_trigger > 0.1) {
            robot.intake.ManualLevel(INTAKE_EXTEND, 1);
            intakeState = IntakeState.INTAKE_EXTEND;
        }
        if (intakeTimer.seconds() > 0.2) {
            robot.intake.setPivot(INTAKE_INT);
            robot.outtake.CloseOuttake(OUTTAKE_CLOSE);
        }
        if (gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND_MID, 1);
            intakeState = IntakeState.OUTTAKE_MID;
        }
        if (gamepad1.dpad_down) {
            robot.outtake.setPivot(OUTTAKE_DUMP);
        }
    }

    private void handleIntakeExtend() {
        if (Math.abs(robot.intake.intakeMotor.getCurrentPosition() - INTAKE_EXTEND) < 10) {
            robot.intake.setPivot(INTAKE_DOWN);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.CloseIntake(CLAW_OPEN);
            intakeTimer.reset();
            intakeState = IntakeState.INTAKE_CLAW_COLLECT_POSITION;
        }
    }

    private void adjustClawPivot() {
        if (gamepad1.dpad_right && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot < 1) {
            clawPivot += 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        } else if (gamepad1.dpad_left && outtakeTimer.seconds() > PIVOT_TIMER && clawPivot > CLAW_VERTICAL) {
            clawPivot -= 0.01;
            robot.intake.setClawPivot(clawPivot);
            outtakeTimer.reset();
        }
    }

    private void adjustClawPosition() {
        if (gamepad1.dpad_down) {
            robot.intake.setClawPivot(CLAW_VERTICAL);
            clawPivot = CLAW_VERTICAL;
            isHorizontal = false;
        } else if (gamepad1.dpad_up) {
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            isHorizontal = true;
        }
    }

    private void handleIntakeClawCollectPosition() {
        if (gamepad1.left_bumper) {
            robot.intake.CloseIntake(CLAW_CLOSE);
            robot.outtake.setPivot(OUTTAKE_COLLECT);
        } else if (gamepad1.right_bumper) {
            robot.intake.OpenIntake(CLAW_OPEN);
            robot.outtake.setPivot(OUTTAKE_COLLECT);
        }

        adjustClawPivot();
        adjustClawPosition();

        if (gamepad1.left_trigger > 0.1) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
            robot.intake.setClawPivot(CLAW_HORIZONTAL);
            clawPivot = CLAW_HORIZONTAL;
            robot.intake.setPivot(INTAKE_INT);
            robot.intake.ManualLevel(INTAKE_LOW, 1);
            intakeState = IntakeState.INTAKE_RETRACT;
            intakeTimer.reset();
        }
    }

    private void handleIntakeRetract() {
        if (robot.intake.intakeLimit.isPressed()) isPressed = true;
        if (isPressed) {
            robot.intake.setPivot(INTAKE_UP);
            if (intakeTimer.seconds() > CLAW_TIMER) {
                robot.intake.OpenIntake(CLAW_OPEN);
                intakeTimer.reset();
                intakeState = IntakeState.INTAKE_START;
            }
        }
    }

    private void handleOuttakeMid() {
        if (gamepad2.cross) {
            robot.outtake.ManualLevel(OUTTAKE_LOW, 0.4);
            intakeState = IntakeState.OUTTAKE_RETRACT;
        }
        robot.outtake.setPivot(OUTTAKE_DUMP);
        if (gamepad2.right_bumper) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        }
        if (Math.abs(robot.outtake.outtakeMotor.getCurrentPosition() - OUTTAKE_EXTEND_MID) < 100 && gamepad2.triangle) {
            robot.outtake.ManualLevel(OUTTAKE_EXTEND, 1);
            intakeState = IntakeState.OUTTAKE_EXTEND;
        }
    }

    private void handleOuttakeExtend() {
        if (gamepad2.cross) {
            robot.outtake.ManualLevel(OUTTAKE_LOW, 0.4);
            intakeState = IntakeState.OUTTAKE_RETRACT;
        }
        robot.outtake.setPivot(OUTTAKE_DUMP);
        if (gamepad2.right_bumper) {
            robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        }
    }

    private void handleOuttakeRetract() {
        robot.outtake.setPivot(OUTTAKE_COLLECT);
        robot.outtake.OpenOuttake(OUTTAKE_OPEN);
        intakeState = IntakeState.INTAKE_START;
    }

    private void updateFollower() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
    }

    @Override
    public void init() {
        robot = new robot(hardwareMap);
        follower = new Follower(hardwareMap);
        intakeTimer.reset();
        outtakeTimer.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        follower.startTeleopDrive();
        initializeRobot();
    }

    @Override
    public void loop() {
        switch (intakeState) {
            case INTAKE_START:
                handleIntakeStart();
                break;
            case INTAKE_EXTEND:
                handleIntakeExtend();
                break;
            case INTAKE_CLAW_COLLECT_POSITION:
                handleIntakeClawCollectPosition();
                break;
            case INTAKE_RETRACT:
                handleIntakeRetract();
                break;
            case OUTTAKE_MID:
                handleOuttakeMid();
                break;
            case OUTTAKE_EXTEND:
                handleOuttakeExtend();
                break;
            case OUTTAKE_RETRACT:
                handleOuttakeRetract();
                break;
            default:
                intakeState = IntakeState.INTAKE_START;
                break;
        }

        updateFollower();
        telemetry.update();
    }
}