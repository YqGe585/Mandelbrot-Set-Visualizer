set_global_assignment -entity "ite_pll" -library "lib_ite_pll" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "ite_pll" -library "lib_ite_pll" -name IP_TOOL_VERSION "18.1"
set_global_assignment -entity "ite_pll" -library "lib_ite_pll" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_ite_pll" -name SPD_FILE [file join $::quartus(sip_path) "ite_pll.spd"]

set_global_assignment -library "lib_ite_pll" -name MISC_FILE [file join $::quartus(sip_path) "ite_pll_sim/ite_pll.vo"]
