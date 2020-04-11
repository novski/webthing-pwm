import smbus


# pylint: disable=bad-whitespace
INA219_ADDRESS                          = 0x40
INA219_BUSNUM                           = 0

INA219_REG_CONFIGURATION                = 0x00
INA219_REG_SHUNT_VOLTAGE                = 0x01
INA219_REG_BUS_VOLTAGE                  = 0x02
INA219_REG_POWER                        = 0x03
INA219_REG_CURRENT                      = 0x04
INA219_REG_CALIBRATION                  = 0x05

CONFIG_RESET                            = 0x8000 # Reset Bit

INA219_CONFIG_BVOLTAGERANGE_MASK        = 0x2000 # Bus Voltage Range Mask
INA219_CONFIG_BVOLTAGERANGE_16V         = 0x0000 #  0-16V Range
INA219_CONFIG_BVOLTAGERANGE_32V         = 0x2000 #  0-32V Range

INA219_CONFIG_GAIN_MASK                 = 0x1800 #  Gain Mask
INA219_CONFIG_GAIN_1_40MV               = 0x0000 #  Gain 1, 40mV Range
INA219_CONFIG_GAIN_2_80MV               = 0x0800 #  Gain 2, 80mV Range
INA219_CONFIG_GAIN_4_160MV              = 0x1000 #  Gain 4, 160mV Range
INA219_CONFIG_GAIN_8_320MV              = 0x1800 #  Gain 8, 320mV Range

INA219_CONFIG_BADCRES_MASK              = 0x0780 #  Bus ADC Resolution Mask
INA219_CONFIG_BADCRES_9BIT              = 0x0080 #  9-bit bus res = 0..511
INA219_CONFIG_BADCRES_10BIT             = 0x0100 #  10-bit bus res = 0..1023
INA219_CONFIG_BADCRES_11BIT             = 0x0200 #  11-bit bus res = 0..2047
INA219_CONFIG_BADCRES_12BIT             = 0x0400 #  12-bit bus res = 0..4097

INA219_CONFIG_SADCRES_MASK              = 0x0078 #  Shunt ADC Resolution and Averaging Mask
INA219_CONFIG_SADCRES_9BIT_1S_84US      = 0x0000 #  1 x 9-bit shunt sample
INA219_CONFIG_SADCRES_10BIT_1S_148US    = 0x0008 #  1 x 10-bit shunt sample
INA219_CONFIG_SADCRES_11BIT_1S_276US    = 0x0010 #  1 x 11-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_1S_532US    = 0x0018 #  1 x 12-bit shunt sample
INA219_CONFIG_SADCRES_12BIT_2S_1060US   = 0x0048 #  2 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_4S_2130US   = 0x0050 #  4 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_8S_4260US   = 0x0058 #  8 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_16S_8510US  = 0x0060 #  16 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_32S_17MS    = 0x0068 #  32 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_64S_34MS    = 0x0070 #  64 x 12-bit shunt samples averaged together
INA219_CONFIG_SADCRES_12BIT_128S_69MS   = 0x0078 #  128 x 12-bit shunt samples averaged together

INA219_CONFIG_MODE_MASK                 = 0x0007 #  Operating Mode Mask
INA219_CONFIG_MODE_POWERDOWN            = 0x0000
INA219_CONFIG_MODE_SVOLT_TRIGGERED      = 0x0001
INA219_CONFIG_MODE_BVOLT_TRIGGERED      = 0x0002
INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED  = 0x0003
INA219_CONFIG_MODE_ADCOFF               = 0x0004
INA219_CONFIG_MODE_SVOLT_CONTINUOUS     = 0x0005
INA219_CONFIG_MODE_BVOLT_CONTINUOUS     = 0x0006
INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x0007


class INA219:

    def __init__(self, busnum=INA219_BUSNUM, address=INA219_ADDRESS):
        self.address = address
        self.bus = smbus.SMBus(busnum)
        self.set_calibration_16V_400mA()

    def read_word(self, register):
        data = self.bus.read_i2c_block_data(self.address, register)
        return data[0] << 8 | data[1]

    def write_word(self, register, data):
        return self.bus.write_i2c_block_data(
            self.address, register, [(data >> 8) & 0xFF, data & 0xFF])

    def calibrate(
        self,
        bus_voltage_range=INA219_CONFIG_BVOLTAGERANGE_32V,
        gain=INA219_CONFIG_GAIN_8_320MV,
        bus_adc=INA219_CONFIG_BADCRES_12BIT,
        shunt_adc=INA219_CONFIG_SADCRES_12BIT_1S_532US,
        mode=INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS,
        cal_value=4096
    ):
        cmd = bus_voltage_range | gain | bus_adc | shunt_adc | mode
        self.write_word(INA219_REG_CALIBRATION, cal_value)
        self.write_word(INA219_REG_CONFIGURATION, cmd)

    def set_calibration_32V_2A(self):
        self.current_divider_mA = 10
        self.power_multiplier_mW = 2
        self.cal_value = 4096
        self.calibrate(
            bus_voltage_range=INA219_CONFIG_BVOLTAGERANGE_32V,
            gain=INA219_CONFIG_GAIN_8_320MV,
            bus_adc=INA219_CONFIG_BADCRES_12BIT,
            shunt_adc=INA219_CONFIG_SADCRES_12BIT_1S_532US,
            mode=INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS,
            cal_value=self.cal_value
        )

    def set_calibration_32V_1A(self):
        self.current_divider_mA = 25
        self.power_multiplier_mW = 0.8
        self.cal_value = 10240
        self.calibrate(
            bus_voltage_range=INA219_CONFIG_BVOLTAGERANGE_32V,
            gain=INA219_CONFIG_GAIN_8_320MV,
            bus_adc=INA219_CONFIG_BADCRES_12BIT,
            shunt_adc=INA219_CONFIG_SADCRES_12BIT_1S_532US,
            mode=INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS,
            cal_value=self.cal_value
        )

    def set_calibration_16V_400mA(self):
        self.current_divider_mA = 20
        self.power_multiplier_mW = 1.0
        self.cal_value = 8192
        self.calibrate(
            bus_voltage_range=INA219_CONFIG_BVOLTAGERANGE_16V,
            gain=INA219_CONFIG_GAIN_1_40MV,
            bus_adc=INA219_CONFIG_BADCRES_12BIT,
            shunt_adc=INA219_CONFIG_SADCRES_12BIT_1S_532US,
            mode=INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS,
            cal_value=self.cal_value
        )

    def get_current_mA(self):
        self.write_word(INA219_REG_CALIBRATION, self.cal_value)
        raw = self.read_word(INA219_REG_CURRENT)
        return raw / self.current_divider_mA

    def get_power_mW(self):
        self.write_word(INA219_REG_CALIBRATION, self.cal_value)
        raw = self.read_word(INA219_REG_POWER)
        return raw * self.power_multiplier_mW

    def get_bus_voltage_mV(self):
        raw = self.read_word(INA219_REG_BUS_VOLTAGE)
        cnvr = True if raw & 0x02 else False
        ovf = True if raw & 0x01 else False
        assert cnvr, 'Conversion is not ready.'
        assert not ovf, 'Power or Current calculations are out of range.'
        return (raw >> 3) * 4

    def get_shunt_voltage_mV(self):
        raw = self.read_word(INA219_REG_SHUNT_VOLTAGE)
        return raw * 0.01