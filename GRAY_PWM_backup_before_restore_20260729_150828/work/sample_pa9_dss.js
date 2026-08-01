importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
var server = null;
var session = null;
var ccxml = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/targetConfigs/MSPM0G3507.ccxml";

function read32(address) {
    return session.memory.readData(Memory.Page.PROGRAM, address, 32, false);
}

try {
    script.setScriptTimeout(30000);
    server = script.getServer("DebugServer.1");
    server.setConfig(ccxml);
    session = server.openSession("Texas Instruments XDS110 USB Debug Probe/CORTEX_M0P");
    session.target.connect();
    session.target.halt();

    var high = 0;
    var low = 0;
    var lastGpioAInput = 0;
    for (var i = 0; i < 200; i++) {
        var gpioAInput = read32(0x400A1380);
        lastGpioAInput = gpioAInput;
        if ((gpioAInput & (1 << 9)) != 0) {
            high++;
        } else {
            low++;
        }
        Thread.sleep(5);
    }

    System.out.println("[PA9-SAMPLE] high=" + high + " low=" + low);
    System.out.println("[GPIOA-DIN] value=" + lastGpioAInput +
        " PA8=" + ((lastGpioAInput >> 8) & 1) +
        " PA9=" + ((lastGpioAInput >> 9) & 1));
    var pincm20 = read32(0x40428050);
    System.out.println("[PA9-SAMPLE] PINCM20=" + pincm20);

    session.memory.writeData(
        Memory.Page.PROGRAM, 0x40428050, pincm20 | 0x00020000, 32);
    System.out.println("[PA9-PULLUP] PINCM20_READBACK=" +
        read32(0x40428050));
    var pullupHigh = 0;
    var pullupLow = 0;
    for (var pullupSample = 0; pullupSample < 100; pullupSample++) {
        var pullupInput = read32(0x400A1380);
        if ((pullupInput & (1 << 9)) != 0) {
            pullupHigh++;
        } else {
            pullupLow++;
        }
        Thread.sleep(5);
    }
    System.out.println("[PA9-PULLUP] high=" + pullupHigh +
        " low=" + pullupLow);

    session.memory.writeData(
        Memory.Page.PROGRAM, 0x40428050,
        (pincm20 & 0xFFFFFFC0) | 0x00020081, 32);
    var gpioModeHigh = 0;
    var gpioModeLow = 0;
    for (var gpioModeSample = 0; gpioModeSample < 100; gpioModeSample++) {
        var gpioModeInput = read32(0x400A1380);
        if ((gpioModeInput & (1 << 9)) != 0) {
            gpioModeHigh++;
        } else {
            gpioModeLow++;
        }
        Thread.sleep(5);
    }
    System.out.println("[PA9-GPIO-PULLUP] high=" + gpioModeHigh +
        " low=" + gpioModeLow +
        " PINCM20=" + read32(0x40428050));
    session.memory.writeData(
        Memory.Page.PROGRAM, 0x40428050, pincm20, 32);

    System.out.println("[PA9-SAMPLE] UART1_STAT=" + read32(0x40101108));
    session.target.runAsynch();
    Thread.sleep(100);
    System.out.println("[PA9-SAMPLE] TARGET_RUNNING");
} finally {
    if (session != null) {
        try {
            if (session.target.isConnected()) {
                session.target.disconnect();
            }
        } catch (error) {
        }
        try {
            session.terminate();
        } catch (error2) {
        }
    }
    if (server != null) {
        try {
            server.stop();
        } catch (error3) {
        }
    }
}
