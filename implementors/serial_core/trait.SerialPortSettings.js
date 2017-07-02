(function() {var implementors = {};
implementors["serial_core"] = [];
implementors["serial_unix"] = ["impl SerialPortSettings for <a class=\"struct\" href=\"serial_unix/struct.TTYSettings.html\" title=\"struct serial_unix::TTYSettings\">TTYSettings</a>",];
implementors["serial_windows"] = ["impl <a class=\"trait\" href=\"serial_core/trait.SerialPortSettings.html\" title=\"trait serial_core::SerialPortSettings\">SerialPortSettings</a> for <a class=\"struct\" href=\"serial_windows/struct.COMSettings.html\" title=\"struct serial_windows::COMSettings\">COMSettings</a>",];

            if (window.register_implementors) {
                window.register_implementors(implementors);
            } else {
                window.pending_implementors = implementors;
            }
        
})()
