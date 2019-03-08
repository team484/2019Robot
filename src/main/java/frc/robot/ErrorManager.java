/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class ErrorManager {
    private static ArrayList<String> errors = new ArrayList<>();
    public static void add(String err) {
        if (errors.contains(err)) {
            return;
        }
        errors.add(err);
        ErrorManager.updateSmartDashboard();
    }

    public static void updateSmartDashboard() {
        SmartDashboard.putStringArray("ErrorLog", errors.toArray(new String[errors.size()]));
        for (int i = 0; i < 5; i++) {
            String text = errors.size() > i ? errors.get(i) : "";
            SmartDashboard.putString("err" + i, text);
        }
        SmartDashboard.putNumber("Errors", errors.size());
    }
}
