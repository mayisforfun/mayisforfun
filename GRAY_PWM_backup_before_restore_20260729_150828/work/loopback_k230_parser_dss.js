importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
var server = null;
var session = null;
var ccxml = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/targetConfigs/MSPM0G3507.ccxml";
var program = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/Debug/GRAY_PWM_backup_before_restore_20260729_150828.out";

function log(message) {
    System.out.println("[K230-LOOPBACK] " + message);
}

function read32(address) {
    return session.memory.readData(Memory.Page.PROGRAM, address, 32, false);
}

function write32(address, data) {
    session.memory.writeData(Memory.Page.PROGRAM, address, data, 32);
}

try {
    script.setScriptTimeout(30000);
    server = script.getServer("DebugServer.1");
    server.setConfig(ccxml);
    session = server.openSession("Texas Instruments XDS110 USB Debug Probe/CORTEX_M0P");
    session.target.connect();
    session.memory.loadProgram(program);
    session.target.restart();
    session.target.runAsynch();
    Thread.sleep(200);
    session.target.halt();

    var ctl0 = read32(0x40101100);
    write32(0x40101100, ctl0 | 0x00000004);
    var frame = [0xAA, 0xAA, 0x01, 0x40, 0x00, 0xF0, 0xFF, 0xFF];
    for (var i = 0; i < frame.length; i++) {
        write32(0x40101120, frame[i]);
        session.target.runAsynch();
        Thread.sleep(20);
        session.target.halt();
    }

    log("frame_count=" + session.expression.evaluate("g_k230_frame_count"));
    log("bad_frame_count=" +
        session.expression.evaluate("g_k230_bad_frame_count"));
    log("error_irq_count=" +
        session.expression.evaluate("g_k230_error_irq_count"));
    log("center_x=" + session.expression.evaluate("g_k230_center_x"));
    log("center_y=" + session.expression.evaluate("g_k230_center_y"));
    log("position_valid=" +
        session.expression.evaluate("g_k230_position_valid"));

    session.memory.loadProgram(program);
    session.target.restart();
    session.target.runAsynch();
    Thread.sleep(100);
    log("NORMAL_PROGRAM_RESTORED_AND_RUNNING");
} finally {
    if (session != null) {
        try {
            if (session.target.isConnected()) {
                session.target.disconnect();
            }
        } catch (error) {
            log("DISCONNECT_ERROR=" + error);
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
