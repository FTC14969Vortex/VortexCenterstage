// AutoBlueFront.java
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBlueBack", group = "Auto")
public class AutoBlueBack extends AutoCommon {

    @Override
    public void setUniqueParameters() {
        // Specific values for AutoBlueFront
        targetAprilTagOffset = 0;
        strafeDirAfterPurPix = -1;
        turnAngleNearBackstage = -93;
        strafeDistAfterPurPix = 27;
        strafeDistAtBackboard = -39;
        strafeDirForParking = -1;
        //-1 for blue, 1 for Red
        redOrBlueSide = -1;
        super.centerTagID = 2;
    }
    
}
