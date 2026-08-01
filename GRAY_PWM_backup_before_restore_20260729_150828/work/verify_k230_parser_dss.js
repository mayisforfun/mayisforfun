importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
var server = null;
var session = null;
var ccxml = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/targetConfigs/MSPM0G3507.ccxml";
var program = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/Debug/GRAY_PWM_backup_before_restore_20260729_150828.out";

function log(message) {
    System.out.println("[K230-VERIFY] " + message);
}

function value(name) {
    return session.expression.evaluate(name);
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
    Thread.sleep(3000);
    session.target.halt();

    var names = [
        "g_sys_ticks",
        "g_uart_rx_count",
        "g_uart_irq_count",
        "g_uart_rx_irq_count",
        "g_uart_last_irq_iidx",
        "g_k230_error_irq_count",
        "g_k230_rx_count",
        "g_k230_frame_count",
        "g_k230_bad_frame_count",
        "g_k230_parse_state",
        "g_k230_position_valid",
        "g_k230_position_fresh",
        "g_k230_center_x",
        "g_k230_center_y",
        "g_k230_last_frame_ticks"
    ];
    for (var i = 0; i < names.length; i++) {
        log(names[i] + "=" + value(names[i]));
    }

    var bytes = [];
    for (var j = 0; j < 8; j++) {
        bytes.push(value("g_k230_raw_last8[" + j + "]"));
    }
    log("g_k230_raw_last8=" + bytes.join(","));

    var ris = session.memory.readData(
        Memory.Page.PROGRAM, 0x40101030, 32, false);
    log("UART1_CPU_INT_RIS=" + ris);
    log("VERIFY_COMPLETED");

    session.target.runAsynch();
    Thread.sleep(100);
    log("TARGET_RUNNING");
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
