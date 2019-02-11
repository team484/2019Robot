/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

/**
 * Add your docs here.
 */
public class VisionTapeResult {
    public boolean isLeftRect;
    public double targetY,targetX, ppi;
    public RotatedRect rect;
    public Point outsideEdge;
    public VisionTapeResult(RotatedRect rect, double targetX, double targetY, double ppi, Point outsideEdge, boolean isLeftRect) {
        this.rect = rect;
        this.targetX = targetX;
        this.targetY = targetY;
        this.ppi = ppi;
        this.outsideEdge = outsideEdge;
        this.isLeftRect = isLeftRect;
    }

    public double getDistance(VisionTapeResult otherResult) {
        return Math.sqrt(Math.pow(otherResult.targetX - targetX, 2)+
        Math.pow(otherResult.targetY - targetY, 2));
    }
}
