package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="AutonTestInBlue", group="Pushbot")
@Disabled
public class AutonTestInBlue extends AutonTestIn {

    public AutonTestInBlue(){
        super.setAllianceColor(BasicAuton.GAME_ALLIANCE_BLUE);
    }

}
