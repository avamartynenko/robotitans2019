package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoHooks {

    Servo hookRight;
    Servo hookLeft;
    HardwareMap hwMap =  null;
    public final int DOWN = 1;
    public final int UP = 2;
    public double LEFT_HOOK_LATCH_POSITION = 0.5;
    public double RIGHT_HOOK_LATCH_POSITION = 0.2;

    public double LEFT_HOOK_HOME_POSITION = 0.2;
    public double RIGHT_HOOK_HOME_POSITION = 0.5;


    public ServoHooks(HardwareMap parentHwMap){

        hookLeft = parentHwMap.get(Servo.class, "hookLeft");
        hookRight = parentHwMap.get(Servo.class, "hookRight");

        hookLeft.setPosition(LEFT_HOOK_HOME_POSITION);
        hookRight.setPosition(RIGHT_HOOK_HOME_POSITION);


    }

    public void latch(){

        hookLeft.setPosition(LEFT_HOOK_LATCH_POSITION);
        hookRight.setPosition(RIGHT_HOOK_LATCH_POSITION);
    }

    public void release(){
        hookLeft.setPosition(LEFT_HOOK_HOME_POSITION);
        hookRight.setPosition(RIGHT_HOOK_HOME_POSITION);
    }


}


