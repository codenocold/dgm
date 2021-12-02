#ifndef __HW_CONFIG_H__
#define __HW_CONFIG_H__

#define SHUNT_RESISTENCE	0.001f								// Resistance of phase current sampling resistor
#define V_SCALE (19.0f * 3.3f / 4095.0f)     					// Bus volts per A/D Count (0.015311 V)
#define I_SCALE (3.3f / 4095.0f) / SHUNT_RESISTENCE / 40.0f		// Amps per A/D Count (0.020147 A)

#define PWM_ARR		3400           	// 25KHz
#define DT			0.00004f		// 40us
#define CURRENT_MEAS_HZ		(1.0f / DT)

#endif
