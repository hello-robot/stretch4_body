"""PIMU EOA output (firmware v0.1.50p14): status bit 21 and restore_eoa_power().

The firmware latches the EOA output as tripped when its overcurrent cut fires and
reports it as bit 21 of the status state word. restore_eoa_power() reuses
actuator_control('eoa', True), the existing RPC_ACTUATOR_ENABLE; no new RPC.
Fake transport, no hardware.
"""
from test_power_periph_easter_egg import PowerPeriph, pimu, status_reply  # noqa: F401


def test_status_bit_21_reads_back_as_eoa_output_tripped(pimu):
    assert pimu.status['eoa_output_tripped'] is False
    pimu.transport.replies.append(status_reply(PowerPeriph.STATE_EOA_OUTPUT_TRIPPED |
                                               PowerPeriph.STATE_RUNSTOP_EVENT))
    pimu.pull_status()
    assert pimu.status['eoa_output_tripped'] is True
    assert pimu.status['runstop_event'] is True
    assert pimu.status['easter_egg_mode'] == 0

    pimu.transport.replies.append(status_reply(PowerPeriph.STATE_EASTER_EGG_MODE))
    pimu.pull_status()
    assert pimu.status['eoa_output_tripped'] is False


def test_bit_21_is_not_another_state_bit():
    others = [v for k, v in vars(PowerPeriph).items()
              if k.startswith('STATE_') and k != 'STATE_EOA_OUTPUT_TRIPPED']
    assert PowerPeriph.STATE_EOA_OUTPUT_TRIPPED == 1 << 21
    assert all(v & (1 << 21) == 0 for v in others)


def test_restore_eoa_power_sends_actuator_enable_for_eoa_on_the_next_push(pimu):
    pimu.restore_eoa_power()
    assert pimu.transport.payloads == []
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_ACTUATOR_ENABLE])
    assert pimu.push_command() is True
    assert pimu.transport.payloads == [[PowerPeriph.RPC_ACTUATOR_ENABLE, 6]]
    # Sent once.
    assert pimu.push_command() is True
    assert len(pimu.transport.payloads) == 1


def test_restore_eoa_power_on_the_robot_client_queues_the_server_command():
    from stretch4_body.robot.robot_client import PowerPeriphClient
    sent = []
    client = PowerPeriphClient.__new__(PowerPeriphClient)
    client._queue_command = lambda **kw: sent.append(kw)
    client.restore_eoa_power()
    assert sent == [{'subsystem': 'power_periph', 'command': 'restore_eoa_power'}]
