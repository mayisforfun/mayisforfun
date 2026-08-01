importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
var server = null;
var session = null;
var ccxml = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/targetConfigs/MSPM0G3507.ccxml";
var program = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/Debug/GRAY_PWM_backup_before_restore_20260729_150828.out";

try {
    script.setScriptTimeout(30000);
    server = script.getServer("DebugServer.1");
    server.setConfig(ccxml);
    session = server.openSession("Texas Instruments XDS110 USB Debug Probe/CORTEX_M0P");
    session.target.connect();
    session.memory.loadProgram(program);
    session.target.restart();
    session.target.runAsynch();
    Thread.sleep(300);
    System.out.println("[RESTORE] PROGRAM_RELOADED_AND_RUNNING");
} finally {
    if (session != null) {
        try {
            if (session.target.isConnected()) {
                session.target.disconnect();
            }
        } catch (error) {
            System.out.println("[RESTORE] DISCONNECT_ERROR=" + error);
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
