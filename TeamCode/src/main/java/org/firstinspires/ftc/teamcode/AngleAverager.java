package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

class AngleAverager {
    public void getAngle(double currentDistance){
        //Math.floor basically rounds down an integer. Both 6.1 and 6.8 would become 6.
        double roundedDownDistance = Math.floor(currentDistance/10.0) * 10;
        double roundedUpDistance = Math.ceil(currentDistance/10.0) * 10;
        double distanceDecimal = .1 * (roundedUpDistance-currentDistance);
        //These retrieve the needed files stored during calibration
        double launchAngleHigh = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(roundedUpDistance+".txt")));
        double launchAngleLow = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile(roundedDownDistance+".txt")));
        double launchAngleReal = distanceDecimal*launchAngleLow + (1-distanceDecimal)*launchAngleHigh;
    }

}
