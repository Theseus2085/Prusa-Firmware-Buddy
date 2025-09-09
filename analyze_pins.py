import re

# Read the pin definition file
with open('src/common/hwio_pindef.h', 'r') as f:
    content = f.read()

print('=== COMPLETE MK4 Pin Analysis ===')

# Extract Marlin pin definitions
marlin_pins = []
marlin_pattern = r'#define MARLIN_PORT_(\w+)\s+MARLIN_PORT_([A-G])'
for match in re.finditer(marlin_pattern, content):
    pin_name = match.group(1)
    port = match.group(2)
    marlin_pins.append((pin_name, port))

marlin_pin_nums = []
pin_num_pattern = r'#define MARLIN_PIN_NR_(\w+)\s+MARLIN_PIN_NR_(\d+)'
for match in re.finditer(pin_num_pattern, content):
    pin_name = match.group(1)
    pin_num = match.group(2)
    marlin_pin_nums.append((pin_name, pin_num))

# Combine Marlin pin info
marlin_combined = {}
for name, port in marlin_pins:
    marlin_combined[name] = {'port': port}
for name, pin_num in marlin_pin_nums:
    if name in marlin_combined:
        marlin_combined[name]['pin'] = pin_num

print('Marlin-defined pins used on MK4:')
for name, info in marlin_combined.items():
    if 'port' in info and 'pin' in info:
        print(f'P{info["port"]}{info["pin"]} - {name}')

print(f'\nTotal Marlin pins: {len([x for x in marlin_combined.values() if "port" in x and "pin" in x])}')

print('\nCommon pins (all boards):')
print('PE11 - fanPrintPwm')
print('PE9 - fanHeatBreakPwm')

print('\n=== FREE PINS ANALYSIS ===')
print('STM32F407 has ports A-G with pins 0-15 each')

# Collect all used pins
all_used = set()

# Add Marlin pins
for name, info in marlin_combined.items():
    if 'port' in info and 'pin' in info:
        all_used.add(f'{info["port"]}{info["pin"]}')

# Add board-specific pins (extracted earlier)
board_pins = [
    'D11', 'D15', 'G4', 'D13', 'D12', 'G3', 'B7', 'G8', 'G2', 'B6',
    'G5', 'G6', 'G7', 'D8', 'G11', 'D9', 'F14', 'A9', 'A10', 'G0',
    'F2', 'E10', 'C8', 'F13', 'E7', 'E3', 'G1'
]
for pin in board_pins:
    all_used.add(pin)

# Add common pins
all_used.add('E11')
all_used.add('E9')

print(f'\nTotal pins used: {len(all_used)}')

# Find free pins
all_ports = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
free_pins = []

for port in all_ports:
    for pin_num in range(16):
        pin_id = f'{port}{pin_num}'
        if pin_id not in all_used:
            free_pins.append(pin_id)

print(f'Total free pins: {len(free_pins)}')
print('\nFree pins by port:')
for port in all_ports:
    port_free = [p for p in free_pins if p.startswith(port)]
    if port_free:
        nums = [int(p[1:]) for p in port_free]
        nums.sort()
        print(f'Port {port}: {nums}')

print('\n=== RECOMMENDED FREE PINS FOR PROJECTS ===')
print('Best options for new functionality:')

# Filter out power/system pins and suggest good general-purpose pins
good_free = []
for pin in free_pins:
    port = pin[0]
    num = int(pin[1:])
    
    # Avoid commonly reserved pins
    if port == 'A' and num in [0, 1]:  # UART
        continue
    if port == 'B' and num in [0, 1]:  # Often used for other functions
        continue
    if port == 'C' and num in [13, 14, 15]:  # Often used for system functions
        continue
    
    good_free.append(pin)

print(f'\nRecommended free pins ({len(good_free)} total):')
for port in all_ports:
    port_good = [p for p in good_free if p.startswith(port)]
    if port_good:
        nums = [int(p[1:]) for p in port_good]
        nums.sort()
        print(f'Port {port}: {nums}')

print('\n=== ADC-CAPABLE FREE PINS ===')
print('These pins can be used for analog sensors:')
adc_pins = []
for pin in good_free:
    port = pin[0]
    num = int(pin[1:])
    
    # STM32F407 ADC pins
    if port == 'A' and num <= 7:
        adc_pins.append(f'{pin} (ADC123_IN{num})')
    elif port == 'B' and num in [0, 1]:
        adc_pins.append(f'{pin} (ADC12_IN{num+8})')
    elif port == 'C' and num <= 5:
        adc_pins.append(f'{pin} (ADC123_IN{num+10})')

for pin in adc_pins:
    print(f'  {pin}')
