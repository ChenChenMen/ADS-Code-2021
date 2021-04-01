from smbus import SMBus

i2cBus = SMBus(2) # the bus 2

## Constants

# I2C device addresses
LIS3MDL_ADDR = 0x1e      # Magnetometer
LPS25H_ADDR = 0x5d      # Barometric pressure sensor
LSM6DS33_ADDR = 0x6b      # Gyrometer / accelerometer

# LSM6DS33 gyroscope and accelerometer control registers
LSM6DS33_CTRL1_XL = 0x10  # Acceleration sensor control
LSM6DS33_CTRL2_G = 0x11  # Angular rate sensor (gyroscope) control

# INT 1 pin control
LSM6DS33_INT1_CTRL = 0x0d

# LSM6DS33 Gyroscope and accelerometer output registers
LSM6DS33_OUTX_L_G = 0x22  # Gyroscope pitch axis (X) output, low byte
LSM6DS33_OUTX_H_G = 0x23  # Gyroscope pitch axis (X) output, high byte
LSM6DS33_OUTY_L_G = 0x24  # Gyroscope roll axis (Y) output, low byte
LSM6DS33_OUTY_H_G = 0x25  # Gyroscope roll axis (Y) output, high byte
LSM6DS33_OUTZ_L_G = 0x26  # Gyroscope yaw axis (Z) output, low byte
LSM6DS33_OUTZ_H_G = 0x27  # Gyroscope yaw axis (Z) output, high byte

LSM6DS33_OUTX_L_XL = 0x28  # Accelerometer pitch axis (X) output, low byte
LSM6DS33_OUTX_H_XL = 0x29  # Accelerometer pitch axis (X) output, high byte
LSM6DS33_OUTY_L_XL = 0x2A  # Accelerometer roll axis (Y) output, low byte
LSM6DS33_OUTY_H_XL = 0x2B  # Accelerometer roll axis (Y) output, high byte
LSM6DS33_OUTZ_L_XL = 0x2C  # Accelerometer yaw axis (Z) output, low byte
LSM6DS33_OUTZ_H_XL = 0x2D  # Accelerometer yaw axis (Z) output, high byte

## enable the accelerometer
i2cBus.write_byte_data(LSM6DS33_ADDR, LSM6DS33_CTRL1_XL, 0x60)
i2cBus.write_byte_data(LSM6DS33_ADDR, LSM6DS33_CTRL9_XL, 0x38)
i2cBus.write_byte_data(LSM6DS33_ADDR, LSM6DS33_INT1_CTRL, 0)

accel_registers = [LSM6DS33_OUTX_L_G, LSM6DS33_OUTX_H_G, \
LSM6DS33_OUTY_L_G, LSM6DS33_OUTY_H_G, LSM6DS33_OUTZ_L_G, LSM6DS33_OUTZ_H_G]

def read_3d_sensor(address, registers):
    """ Return a vector with the combined raw signed 16 bit values
        of the output registers of a 3d sensor.
    """

    # Read register outputs and combine low and high byte values
    x_low = read_register(address, registers[0])
    x_hi = read_register(address, registers[1])
    y_low = read_register(address, registers[2])
    y_hi = read_register(address, registers[3])
    z_low = read_register(address, registers[4])
    z_hi = read_register(address, registers[5])

    x_val = combine_signed_lo_hi(x_low, x_hi)
    y_val = combine_signed_lo_hi(y_low, y_hi)
    z_val = combine_signed_lo_hi(z_low, z_hi)

    return [x_val, y_val, z_val]

def read_register(address, register):
	""" Read a single I2C register. """
    return i2cBus.read_byte_data(address, register)

def combine_lo_hi(lo_byte, hi_byte):
    """ Combine low and high bytes to an unsigned 16 bit value. """
    return (hi_byte << 8) | lo_byte

def combine_signed_lo_hi(lo_byte, hi_byte):
    """ Combine low and high bytes to a signed 16 bit value. """
    combined = combine_lo_hi(lo_byte, hi_byte)
    return combined if combined < 32768 else (combined - 65536)

def combine_xlo_lo_hi(xlo_byte, lo_byte, hi_byte):
    """ Combine extra low, low, and high bytes to an unsigned
        24 bit value.
    """
    return (xlo_byte | lo_byte << 8 | hi_byte << 16)

def combine_signed_xlo_lo_hi(xlo_byte, lo_byte, hi_byte):
    """ Combine extra low, low, and high bytes to a signed 24 bit value. """
    combined = combine_xlo_lo_hi(xlo_byte, lo_byte, hi_byte)
    return combined if combined < 8388608 else (combined - 16777216)

def get_accelerometer_raw():
    """ Return a 3D vector of raw accelerometer data.
    """

    # Check if accelerometer has been enabled
    if not is_accel_enabled:
        raise(Exception('Accelerometer is not enabled!'))

    return read_3d_sensor(LSM6DS33_ADDR, accel_registers)

def get_accelerometer_g_forces():
        """ Return a 3D vector of the g forces measured by the accelerometer"""
    [x_val, y_val, z_val] = get_accelerometer_raw()

    x_val = (x_val * ACCEL_CONVERSION_FACTOR) / 1000
    y_val = (y_val * ACCEL_CONVERSION_FACTOR) / 1000
    z_val = (z_val * ACCEL_CONVERSION_FACTOR) / 1000

    return [x_val, y_val, z_val]

while True:
	print (get_accelerometer_g_forces())