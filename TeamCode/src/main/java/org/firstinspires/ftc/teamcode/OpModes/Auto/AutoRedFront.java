// AutoBlueFront.java
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRedFront", group = "Auto")
public class AutoRedFront extends AutoCommon {

    @Override
    public void setUniqueParameters() {
        // Specific values for AutoBlueFront
        targetAprilTagOffset = 3;
        strafeDirAfterPurPix = 1;
        turnAngleNearBackstage = 95;
        strafeDistAfterPurPix = 92;
        strafeDistAtBackboard = -20;
        strafeDirForParking = 1;
        //-1 for blue, 1 for Red
        redOrBlueSide = 1;
        centerTagID = 5;
    }

}
