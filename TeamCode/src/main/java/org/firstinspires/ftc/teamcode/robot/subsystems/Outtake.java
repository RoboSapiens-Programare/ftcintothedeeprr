package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    public Servo pivotOut1, pivotOut2, outtake;
    public DcMotorEx outtakeMotor;

    public Outtake(HardwareMap hardwareMap){
        pivotOut1 = hardwareMap.get(Servo.class, "pivotOut1");
        pivotOut2 = hardwareMap.get(Servo.class, "pivotOut2");
        outtake = hardwareMap.get(Servo.class, "outtake");

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPivot(double position){
        pivotOut1.setPosition(position);
        pivotOut2.setPosition(position);
    }

    public void ManualLevel(int ManualTarget, double power) {
        outtakeMotor.setTargetPosition(ManualTarget);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if(outtakeMotor.getCurrentPosition() < ManualTarget)
        {
            outtakeMotor.setPower(power);
        }
        else
        {
            outtakeMotor.setPower(-power);
        }
    }

    public void OpenOuttake(double position){
        outtake.setPosition(position);
    }

    public void CloseOuttake(double position){
        outtake.setPosition(position);
    }




}
