package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FrogTongue {

    DcMotor frogTongue;
    HardwareMap hwMap =  null;
    public final int EXTEND = 1;
    public final int RETRACT = 2;


    public FrogTongue(HardwareMap parentHwMap) {

        frogTongue = parentHwMap.get(DcMotor.class, "frogTongue");
    }


    private void setDirection(int direction){

        switch (direction) {
            case EXTEND:
                frogTongue.setDirection(DcMotor.Direction.FORWARD);
                break;

            case RETRACT:
                frogTongue.setDirection(DcMotor.Direction.REVERSE);
                break;
        }

    }

    public void setPower(double speed){

        frogTongue.setPower(speed);

    }




}

