// AutoBlueFront.java
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoBlueFront", group = "Auto")
public class AutoBlueFront extends AutoCommon {

    @Override
    public void setUniqueParameters() {
        // Specific values for AutoBlueFront
        targetAprilTagOffset = 0;
        strafeDirAfterPurPix = "Right";
        turnDirNearBackstage = "CCW";
        strafeDistAfterPurPix = 96;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Call the initialization method
        setUniqueParameters();
        // Now call the parent class's runOpMode method
        super.runOpMode();
    }
}
