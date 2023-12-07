// AutoBlueFront.java
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedBack", group = "Auto")
public class AutoRedBack extends AutoCommon {

    @Override
    public void setUniqueParameters() {
        // Specific values for AutoBlueFront
        targetAprilTagOffset = 3;
        strafeDirAfterPurPix = 1;
        turnAngleNearBackstage = 95;
        strafeDistAfterPurPix = 26;
        strafeDistAtBackboard = 35;
        strafeDirForParking = 1;
        //-1 for blue, 1 for Red
        redOrBlueSide = 1;
        DELIVERY_DISTANCE = 20;
        super.centerTagID = 5;

    }

}
