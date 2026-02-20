# FNIRSI DPS-150 Power Supply

port := env("DPS150_PORT", "/dev/cu.usbmodem065AD9D205B31")
py := "python3 dps150.py -p " + port

# Show device info
info:
    {{py}} info

# Full state dump (JSON)
state:
    {{py}} state

# Read measurements
voltage:
    {{py}} voltage

current:
    {{py}} current

power:
    {{py}} power

input-voltage:
    {{py}} input-voltage

temperature:
    {{py}} temperature

# Set voltage setpoint (does not enable output)
set-voltage volts:
    {{py}} set-voltage {{volts}}

# Set current limit (does not enable output)
set-current amps:
    {{py}} set-current {{amps}}

# Set voltage + current and enable output
set-output volts amps:
    {{py}} set-output {{volts}} {{amps}}

# Enable output
on:
    {{py}} on

# Disable output
off:
    {{py}} off

# Protection thresholds
set-ovp volts:
    {{py}} set-ovp {{volts}}

set-ocp amps:
    {{py}} set-ocp {{amps}}

set-opp watts:
    {{py}} set-opp {{watts}}

set-otp celsius:
    {{py}} set-otp {{celsius}}

set-lvp volts:
    {{py}} set-lvp {{volts}}

# Presets
presets:
    {{py}} presets

set-preset n volts amps:
    {{py}} set-preset {{n}} {{volts}} {{amps}}

# Display / sound
set-brightness level:
    {{py}} set-brightness {{level}}

set-volume level:
    {{py}} set-volume {{level}}

# Metering
start-metering:
    {{py}} start-metering

stop-metering:
    {{py}} stop-metering

# Voltage sweep
sweep-voltage start stop step hold current_limit:
    {{py}} sweep-voltage {{start}} {{stop}} {{step}} {{hold}} {{current_limit}}

# Current sweep
sweep-current start stop step hold voltage:
    {{py}} sweep-current {{start}} {{stop}} {{step}} {{hold}} {{voltage}}

# Restart device
restart:
    {{py}} restart
