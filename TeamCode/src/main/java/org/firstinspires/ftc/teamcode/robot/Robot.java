package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;

public class Robot {
    private boolean initialize;
    public SampleMecanumDrive drive;
    public Intake intake;
    public Outtake outtake;


    public Robot(HardwareMap hardwareMap){
        initialize = true;
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        initialize = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isInitialize() {return initialize;}
}
