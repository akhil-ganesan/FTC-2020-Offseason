package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.legacy.states.StateMachine;

public abstract class Subsystem {

    public abstract void init(HardwareMap ahMap);

    public abstract StateMachine getStateMachine();

}
