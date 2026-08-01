importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);
importPackage(Packages.java.lang);

var script = ScriptingEnvironment.instance();
var server = null;
var session = null;

var ccxml = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/targetConfigs/MSPM0G3507.ccxml";
var program = "C:/Users/17570/workspace_ccstheia/GRAY_PWM_backup_before_restore_20260729_150828/Debug/GRAY_PWM_backup_before_restore_20260729_150828.out";

function log(message) {
    System.out.println("[K230-DSS] " + message);
}

function value(name) {
    try {
        return session.expression.evaluate(name);
    } catch (error) {
        return "<read failed: " + error + ">";
    }
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
    log("Loading target configuration");
    server.setConfig(ccxml);

    log("Opening debug session");
    session = server.openSession("Texas Instruments XDS110 USB Debug Probe/CORTEX_M0P");

    log("Connecting to target");
    session.target.connect();
    log("CONNECTED=" + session.target.isConnected());

    log("Loading program");
    session.memory.loadProgram(program);
    log("PROGRAM_LOADED");

    session.target.restart();
    log("Sampling target in ten 250 ms windows");
    for (var sample = 0; sample < 10; sample++) {
        session.target.runAsynch();
        Thread.sleep(250);
        session.target.halt();
        var bytes = [];
        for (var sampleByte = 0; sampleByte < 8; sampleByte++) {
            bytes.push(value("g_uart_rx_buffer[" + sampleByte + "]"));
        }
        log("SAMPLE[" + sample + "] count=" + value("g_uart_rx_count") +
            " index=" + value("g_uart_rx_index") +
            " ring=" + bytes.join(","));
    }
    log("TARGET_HALTED");

    var names = [
        "g_sys_ticks",
        "g_uart_rx_count",
        "g_uart_irq_count",
        "g_uart_rx_irq_count",
        "g_uart_last_irq_iidx",
        "g_uart_rx_data",
        "g_uart_rx_index",
        "g_k230_irq_count",
        "g_k230_rx_irq_count",
        "g_k230_error_irq_count",
        "g_k230_last_irq_iidx",
        "g_k230_rx_count",
        "g_k230_raw0",
        "g_k230_raw1",
        "g_k230_raw2",
        "g_k230_raw3",
        "g_k230_raw4",
        "g_k230_raw5",
        "g_k230_raw6",
        "g_k230_raw7"
    ];

    for (var i = 0; i < names.length; i++) {
        log(names[i] + "=" + value(names[i]));
    }

    for (var j = 0; j < 8; j++) {
        log("g_uart_rx_buffer[" + j + "]=" + value("g_uart_rx_buffer[" + j + "]"));
    }

    var registers = [
        ["CLKDIV", 0x40101000],
        ["CLKSEL", 0x40101008],
        ["CPU_INT_IMASK", 0x40101028],
        ["CPU_INT_RIS", 0x40101030],
        ["CPU_INT_MIS", 0x40101038],
        ["CTL0", 0x40101100],
        ["LCRH", 0x40101104],
        ["STAT", 0x40101108],
        ["IBRD", 0x40101110],
        ["FBRD", 0x40101114]
    ];
    for (var registerIndex = 0; registerIndex < registers.length; registerIndex++) {
        log("UART1_" + registers[registerIndex][0] + "=" +
            read32(registers[registerIndex][1]));
    }

    log("Scanning common UART baud rates");
    var ctl0 = read32(0x40101100);
    var baudRates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600];
    for (var baudIndex = 0; baudIndex < baudRates.length; baudIndex++) {
        var baud = baudRates[baudIndex];
        var divisor = 40000000 / (16 * baud);
        var integerDivisor = Math.floor(divisor);
        var fractionalDivisor = Math.floor(((divisor - integerDivisor) * 64) + 0.5);
        if (fractionalDivisor >= 64) {
            integerDivisor++;
            fractionalDivisor = 0;
        }

        write32(0x40101100, ctl0 & 0xFFFFFFFE);
        write32(0x40101110, integerDivisor);
        write32(0x40101114, fractionalDivisor);
        write32(0x40101100, ctl0);

        session.target.runAsynch();
        Thread.sleep(120);
        session.target.halt();
        read32(0x40101124);
        write32(0x40101048, 0x0002001F);
        session.expression.evaluate("g_uart_rx_count = 0");
        session.expression.evaluate("g_uart_irq_count = 0");
        session.expression.evaluate("g_uart_rx_irq_count = 0");
        session.expression.evaluate("g_uart_rx_index = 0");
        for (var clearIndex = 0; clearIndex < 8; clearIndex++) {
            session.expression.evaluate("g_uart_rx_buffer[" + clearIndex + "] = 0");
        }

        session.target.runAsynch();
        Thread.sleep(500);
        session.target.halt();
        var baudBytes = [];
        for (var baudByte = 0; baudByte < 8; baudByte++) {
            baudBytes.push(value("g_uart_rx_buffer[" + baudByte + "]"));
        }
        log("BAUD[" + baud + "] div=" + integerDivisor + "/" +
            fractionalDivisor + " count=" + value("g_uart_rx_count") +
            " ris=" + read32(0x40101030) +
            " ring=" + baudBytes.join(","));
    }

    log("PC=" + session.memory.readRegister("PC"));
    log("DEBUG_COMPLETED");
} catch (error) {
    log("ERROR=" + error);
    throw error;
} finally {
    if (session != null) {
        try {
            if (session.target.isConnected()) {
                session.target.disconnect();
            }
        } catch (disconnectError) {
            log("DISCONNECT_ERROR=" + disconnectError);
        }
        try {
            session.terminate();
        } catch (terminateError) {
            log("TERMINATE_ERROR=" + terminateError);
        }
    }
    if (server != null) {
        try {
            server.stop();
        } catch (stopError) {
            log("SERVER_STOP_ERROR=" + stopError);
        }
    }
}
