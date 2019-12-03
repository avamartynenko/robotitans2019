package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="AutonTestInWPullBlue", group="Pushbot")
@Disabled
public class AutonTestInWPullBlue extends AutonTestInWPull {
    public AutonTestInWPullBlue(){
        super.setAllianceColor(BasicAuton.GAME_ALLIANCE_BLUE);
    }
}
