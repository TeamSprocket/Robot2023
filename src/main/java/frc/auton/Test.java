package frc.auton;

import java.io.File;
import java.nio.file.FileSystem;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public  class Test extends AbstractGuiAuto {
    
    public Test(){
        super(new File(Filesystem.getDeployDirectory().getPath()+"/auto/Test40.json"));
    }
}