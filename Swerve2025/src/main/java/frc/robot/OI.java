// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI extends XboxController {
    private static OI instance = null;
    public OI (int port) {
        super(port);
    }

    public static OI getInstance() {
        if(instance == null) {
            instance = new OI(0);
        }
        return instance;
    }
}
