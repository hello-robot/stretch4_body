#!/usr/bin/env python3

import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()
import click
import sys

parser=argparse.ArgumentParser(description='Check the battery status')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-v", "--verbose", help="Also print the PIMU power diagnostics (BMS alarms, MOS, temperatures, link)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph


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
    if not battery_is_charging:
        click.secho(f'Battery Charger: Not Charging', fg='yellow')
    if battery_is_charging:
        click.secho(f'Battery Charger: Charging', fg='green')
        click.secho(f'Battery Charger Current: {current_charge:.2f} A', fg='green')

    if args.verbose:
        if args.direct:
            p.pull_power_diagnostics()
        d = p.status['power_diagnostics']
        click.secho('######## Power Diagnostics ##########', fg='cyan')
        if not d['_valid']:
            click.secho('Not available (needs PIMU firmware hello-pimu2.v0.1.18p13 or later)', fg='yellow')
        else:
            click.secho(f"BMS sample age: {d['bms_sample_age_s']} s, valid replies {d['bms_valid_replies']}, "
                        f"CRC errors {d['bms_crc_errors']}, timeouts {d['bms_timeouts']}, RX overflows {d['bms_rx_overflows']}")
            click.secho(f"BMS alarms: {d['bms_alarms']}")
            click.secho(f"BMS MOS: {d['bms_mos_states']}")
            click.secho(f"BMS temperatures (C): cells {d['battery_cell_temp_1_c']} {d['battery_cell_temp_2_c']} "
                        f"{d['battery_cell_temp_3_c']}, ambient {d['battery_ambient_temp_c']}, MOSFET {d['battery_mosfet_temp_c']}")
            click.secho(f"Charger enable command: {d['charger_enable_command']}, expected active: {d['charger_expected_active']}, "
                        f"charger current {d['charger_current_ma']} mA")
            click.secho(f"ESP status age: {d['esp_status_age_s']} s, packets {d['esp_rx_packets']}, CRC errors {d['esp_crc_errors']}")

    print("")


battery_check()
    
