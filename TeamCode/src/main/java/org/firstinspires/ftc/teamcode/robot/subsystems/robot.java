package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.drive.roadRunner.SampleMecanumDrive;

public class robot {
    private boolean initialize;
    public SampleMecanumDrive drive;
    public org.firstinspires.ftc.teamcode.robot.subsystems.intake intake;
    public org.firstinspires.ftc.teamcode.robot.subsystems.outtake outtake;


    public robot(HardwareMap hardwareMap){
        initialize = true;
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new intake(hardwareMap);
        outtake = new outtake(hardwareMap);
        initialize = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isInitialize() {return initialize;}
}
