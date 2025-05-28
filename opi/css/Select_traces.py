# from org.csstudio.opibuilder.scriptUtil import PVUtil

graph = display.getWidget("Oscilloscope Display").setPropertyValue

check1 = display.getWidget("Check_Box_CH1").getValue()
check2 = display.getWidget("Check_Box_CH2").getValue()
check3 = display.getWidget("Check_Box_CH3").getValue()
check4 = display.getWidget("Check_Box_CH4").getValue()

if (check1 == 0):
	graph("trace_0_visible", "false")
else:
	graph("trace_0_visible", "true")
	
if (check2 == 0):
	graph("trace_1_visible", "false")
else:
	graph("trace_1_visible", "true")

if (check3 == 0):
	graph("trace_2_visible", "false")
else:
	graph("trace_2_visible", "true")

if (check4 == 0):
	graph("trace_3_visible", "false")
else:
	graph("trace_3_visible", "true")
