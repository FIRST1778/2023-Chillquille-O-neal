# Do this stuff, Maya

1. Go to [THIS](https://downloads.limelightvision.io/software/LimelightHardwareManagerSetup2_0_10.exe) link and run when done downloading.
   1. Follow the instructions to install the Limelight Hardware Manager
2. Go to [THIS](https://github.com/raspberrypi/usbboot/releases/download/windows-v1.1/rpiboot_setup.exe) link and run when done downloading.
   1. Follow the instructions to install RPIBoot (for mounting the Systemcore)
3. Go to [THIS](https://github.com/LimelightVision/systemcore-os-public/releases/download/limelightosr-2027.0.0-alpha13-367/limelightsystemcorecm5-limelightosr-2027.0.0-alpha13.zip) link.
   1. Do NOT extract this ZIP.   
4. Open the Windows Start menu and search for rpiboot. You should find something called rpiboot - Mass Storage Gadget.
   1. Run this as administrator
5. Turn off the robot.
   1. While off, plug the robot in to the laptop via USB C.
   2. Turn the robot on
6. Don't do anything until RPIBoot closes
7. Open the Windows Start menu and search for limelight. You should find something called Limelight Hardware Manager.
   1. Run this as administrator
8. Navigate to the Flash OS page and click on Select OS
   1. DON'T click Downloads in the left bar
   2. Click on This PC > Local Disk (C:) > Users > [your username here] > Downloads.
   3. You should see something called `limelightsystemcorecm5-limelightosr-2027.0.0-alpha13`
   4. Select this ZIP file
9. Click Select Device
   1. You should only see one option (if not then ping me)
   2. Select this device
10. Click Flash Device
11. Go to IntellIJ and run this command in the terminal `gradlew clean deploy`
12. Open the Driver Station and tell me what you see