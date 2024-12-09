package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class robot {
    private boolean initialize;
    public intake intake;
    public outtake outtake;

    public robot(HardwareMap hardwareMap){
        initialize = true;
        intake = new intake(hardwareMap);
        outtake = new outtake(hardwareMap);
        initialize = false;
    }

    public boolean isInitialize() {return initialize;}
}
