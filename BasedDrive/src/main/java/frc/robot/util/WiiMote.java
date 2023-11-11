package frc.robot.util;
import java.util.ArrayList;
import java.awt.*;
import javax.swing.*;
import wiiremotej.*;
import wiiremotej.event.*;
import javax.sound.sampled.*;
import java.io.*;

public class WiiMote {
    private static boolean accelerometerSource = true; //true = wii remote, false = nunchuk
    private static boolean lastSource = true;
    
    private static boolean mouseTestingOn;
    private static int status = 0;
    private static int accelerometerStatus = 0;
    private static int analogStickStatus = 0;
    private static JFrame mouseTestFrame;
    private static JPanel mouseTestPanel;
    
    private WiiRemote remote;
    private static JFrame graphFrame;
    private static JPanel graph;
    private static int[][] pixels;
    private static int t = 0;
    private static int x = 0;
    private static int y = 0;
    private static int z = 0;
    
    private static int lastX = 0;
    private static int lastY = 0;
    private static int lastZ = 0;
    
    private static PrebufferedSound prebuf;


    public WiiMote(WiiRemote remote)
    {
        this.remote = remote;
    }
    
    public void disconnected()
    {
        System.out.println("Remote disconnected... Please Wii again.");
        System.exit(0);
    }
    
    public void statusReported(WRStatusEvent evt)
    {
        System.out.println("Battery level: " + (double)evt.getBatteryLevel()/2+ "%");
        System.out.println("Continuous: " + evt.isContinuousEnabled());
        System.out.println("Remote continuous: " + remote.isContinuousEnabled());
    }
    
    public void IRInputReceived(WRIREvent evt)
    {
        /*for (IRLight light : evt.getIRLights())
        {
            if (light != null)
            {
                System.out.println("X: "+light.getX());
                System.out.println("Y: "+light.getY());
            }
        }*/
        
    }
    
    public void accelerationInputReceived(WRAccelerationEvent evt)
    {
        //System.out.println("R: " + evt.getRoll());
        //System.out.println("P: " + evt.getPitch());
        if (accelerometerSource)
        {
            lastX = x;
            lastY = y;
            lastZ = z;
            
            x = (int)(evt.getXAcceleration()/5*300)+300;
            y = (int)(evt.getYAcceleration()/5*300)+300;
            z = (int)(evt.getZAcceleration()/5*300)+300;
            
            t++;
            
            graph.repaint();
        }
        
        /*System.out.println("---Acceleration Data---");
        System.out.println("X: " + evt.getXAcceleration());
        System.out.println("Y: " + evt.getYAcceleration());
        System.out.println("Z: " + evt.getZAcceleration());
        */
    }
    
    public void extensionInputReceived(WRExtensionEvent evt)
    {
        if (evt instanceof WRNunchukExtensionEvent)
        {
            WRNunchukExtensionEvent NEvt = (WRNunchukExtensionEvent)evt;
            
            if (!accelerometerSource)
            {
                WRAccelerationEvent AEvt = NEvt.getAcceleration();
                lastX = x;
                lastY = y;
                lastZ = z;
                
                x = (int)(AEvt.getXAcceleration()/5*300)+300;
                y = (int)(AEvt.getYAcceleration()/5*300)+300;
                z = (int)(AEvt.getZAcceleration()/5*300)+300;
                
                t++;
                
                graph.repaint();
            }
            
            if (NEvt.wasReleased(WRNunchukExtensionEvent.C))System.out.println("Jump...");
            if (NEvt.wasPressed(WRNunchukExtensionEvent.Z))System.out.println("And crouch.");
        }
        else if (evt instanceof WRGuitarExtensionEvent)
        {
            WRGuitarExtensionEvent GEvt = (WRGuitarExtensionEvent)evt;
            if (GEvt.wasPressed(WRGuitarExtensionEvent.MINUS))System.out.println("Minus!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.PLUS))System.out.println("Plus!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.STRUM_UP))System.out.println("Strum up!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.YELLOW))System.out.println("Yellow!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.GREEN))System.out.println("Green!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.BLUE))System.out.println("Blue!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.RED))System.out.println("Red!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.ORANGE))System.out.println("Orange!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.STRUM_DOWN))System.out.println("Strum down!");
            if (GEvt.wasPressed(WRGuitarExtensionEvent.GREEN+WRGuitarExtensionEvent.RED))
            {
                System.out.println("Whammy bar: " + GEvt.getWhammyBar());
                AnalogStickData AS = GEvt.getAnalogStickData();
                System.out.println("Analog- X: " + AS.getX() + " Y: " + AS.getY());
            }
        }
    }
    
    public void extensionConnected(WiiRemoteExtension extension)
    {
        System.out.println("Extension connected!");
        try
        {
            remote.setExtensionEnabled(true);
        }catch(Exception e){e.printStackTrace();}
    }
    
    public void extensionPartiallyInserted()
    {
        System.out.println("Extension partially inserted. Push it in more next time!");
    }
    
    public void extensionUnknown()
    {
        System.out.println("Extension unknown. Did you try to plug in a toaster or something?");
    }
    
    public void extensionDisconnected(WiiRemoteExtension extension)
    {
        System.out.println("Extension disconnected. Why'd you unplug it, eh?");
    }
    
    public void buttonInputReceived(WRButtonEvent evt)
    {
        /*
        if (evt.wasPressed(WRButtonEvent.TWO))System.out.println("2");
        if (evt.wasPressed(WRButtonEvent.ONE))System.out.println("1");
        if (evt.wasPressed(WRButtonEvent.B))System.out.println("B");
        if (evt.wasPressed(WRButtonEvent.A))System.out.println("A");
        if (evt.wasPressed(WRButtonEvent.MINUS))System.out.println("Minus");
        if (evt.wasPressed(WRButtonEvent.HOME))System.out.println("Home");
        if (evt.wasPressed(WRButtonEvent.LEFT))System.out.println("Left");
        if (evt.wasPressed(WRButtonEvent.RIGHT))System.out.println("Right");
        if (evt.wasPressed(WRButtonEvent.DOWN))System.out.println("Down");
        if (evt.wasPressed(WRButtonEvent.UP))System.out.println("Up");
        if (evt.wasPressed(WRButtonEvent.PLUS))System.out.println("Plus");
        /**/
    }
}
