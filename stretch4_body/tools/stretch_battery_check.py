#!/usr/bin/env python3

import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()
import click
import sys

parser=argparse.ArgumentParser(description='Check the battery status')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

# With the adapter in, the power board stops the charger once the battery
# reads 99% and restarts it below 95%, so it isn't charging in between
CHARGER_RESTART_SOC = 95


def battery_check():
    p = PowerPeriph()
    if not p.startup():
        click.secho('Pimu comms not available', fg='yellow')
        sys.exit()
    p.pull_status()
    battery_voltage = p.status['voltage']
    battery_soc = p.status['battery_soc']
    battery_soh = p.status['battery_soh']
    adapter_voltage_present = p.status['adapter_voltage_present']
    adapter_fault = p.status['adapter_fault']
    adapter_connected = p.status['adapter_connected']
    battery_is_charging = p.status['charger_is_charging']
    current_charge = p.status['current_charge']
    current_battery = p.status['battery_current']

    click.secho('######## Stretch Battery Information ##########', fg='cyan')
    # The battery values come from the BMS. When it isn't answering they are NaN:
    # show a dash rather than a number that can't be trusted
    battery_status = p.status.get('battery_status', 'UNKNOWN')
    if battery_status == 'BMS_LOST':
        click.secho("Battery: NOT RESPONDING. Values can't be trusted. Check the battery and its cable.", fg='red')
    elif battery_status == 'NOT_DETECTED':
        click.secho('Battery: NOT DETECTED. Running from the adapter.', fg='yellow')
    elif battery_status == 'UNKNOWN':
        click.secho('Battery: status not reported. Update the power board firmware to tell when the values are stale.')
    if not p.status.get('battery_valid', True):
        for name in ('Battery Remaining Capacity', 'Battery Voltage', 'Battery Health Percentage', 'Battery Current Supplied'):
            click.secho(f'{name}: -', fg='bright_black')
    else:
        if battery_soc > 20:
            click.secho(f'Battery Remaining Capacity: {battery_soc}%', fg='green')
            click.secho(f'Battery Voltage: {battery_voltage:.2f} V', fg='green')
        if 10 < battery_soc <= 20:
            click.secho(f'Battery Remaining Capacity: {battery_soc}%', fg='yellow')
            click.secho(f'Battery Voltage: {battery_voltage:.2f} V', fg='yellow')
        if battery_soc <= 10 and not battery_is_charging:
            click.secho(f'Battery Remaining Capacity: {battery_soc}% Please Plug In The Charger', fg='red')
            click.secho(f'Battery Voltage: {battery_voltage:.2f}V', fg='red')
        elif battery_soc <= 10 and battery_is_charging:
            click.secho(f'Battery Remaining Capacity: {battery_soc}%', fg='red')
            click.secho(f'Battery Voltage: {battery_voltage:.2f}V', fg='red')

        if battery_soh >= 75:
            click.secho(f'Battery Health Percentage: {battery_soh}%', fg='green')
        if battery_soh < 75:
            click.secho(f'Battery Health Percentage: {battery_soh}%', fg='yellow')

        click.secho(f'Battery Current Supplied: {current_battery:.2f} A', fg='green')

    click.secho('######## Adapter Information ##########', fg='cyan')
    if not adapter_voltage_present:
        click.secho(f'Adapter voltage: not present', fg='yellow')
    if adapter_voltage_present:
        click.secho(f'Adapter voltage: present', fg='green')
    if not adapter_fault:
        click.secho(f'Adapter fault: not present', fg='green')
    if adapter_fault:
        click.secho(f'Adapter fault: present', fg='red')
    if not adapter_connected:
        click.secho(f'Adapter (physical switch):  not connected', fg='yellow')
    if adapter_connected:
        click.secho(f'Adapter (physical switch): connected', fg='green')
    
    click.secho('######## Charger Information ##########', fg='cyan')
    if battery_is_charging:
        click.secho(f'Battery Charger: Charging', fg='green')
        click.secho(f'Battery Charger Current: {current_charge:.2f} A', fg='green')
    elif not adapter_voltage_present:
        click.secho(f'Battery Charger: Not Charging. Adapter voltage not present', fg='yellow')
    elif battery_soc >= CHARGER_RESTART_SOC:
        click.secho(f'Battery Charger: Not Charging. Battery near full', fg='green')
    else:
        # Briefly while the power board restarts the charger; otherwise a fault
        click.secho(f'Battery Charger: Not Charging. Charger restarting or fault', fg='yellow')
    
    print("")


battery_check()
    
