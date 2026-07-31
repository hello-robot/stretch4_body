from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader


def test_pixart_reader_preserves_bounded_firmware_diagnostics(monkeypatch):
    timestamps = iter([100.0, 101.0, 102.0, 103.0, 104.0, 105.0])
    monkeypatch.setattr(
        "stretch4_body.subsystem.line_sensor.pixart_j3_reader.time.time",
        lambda: next(timestamps),
    )
    reader = PixartJ3Reader()

    assert reader._process_oob_line("Firmware hello-pixart-j3.v0.1.0p5-diag")
    assert reader._process_oob_line(
        "I2C_ADDR_PROBE b1 addr=0x48 ack=0 sercom=1 dmac=0 timeout=0 "
        "status=0x0414 intflag=0x80 dmac_ch=255 dmac_flags=0x00 sda=1 scl=1"
    )
    assert reader._process_oob_line(
        "I2C DEVICE INIT FAILED b1d1 addr=0x48 step=0 reg=0x7f val=0x00 "
        "sercom=1 dmac=0 timeout=0 status=0x0414 iflag=0x80 dch=255 "
        "dflag=0x00 sda=1 scl=1"
    )
    assert reader._process_oob_line(
        "I2C DEVICE INIT FAILED, b1d1, addr=0x48"
    )
    assert reader._process_oob_line(
        "SENSOR FRAME TIMEOUT, b3d0, state=S_POLLING(2), age_us=122000, "
        "frame=91, count=7"
    )
    assert reader._process_oob_line(
        "SENSOR_INT levels=101011 transitions=1029,1028,1029,0,1029,0"
    )

    status = reader.status
    assert status["firmware_version"] == "hello-pixart-j3.v0.1.0p5-diag"
    assert status["diagnostic_sequence"] == 6
    assert status["last_diagnostic_time"] == 105.0

    probe = status["i2c_address_probes"]["b1@0x48"]
    assert probe["ack"] is False
    assert probe["status"] == 0x0414
    assert probe["intflag"] == 0x80
    assert probe["sda"] == 1
    assert probe["scl"] == 1

    init_error = status["i2c_init_errors"]["b1d1@0x48"]
    assert init_error["count"] == 2
    assert init_error["step"] == 0
    assert init_error["reg"] == 0x7F
    assert init_error["status"] == 0x0414

    timeout = status["frame_timeouts"]["b3d0:S_POLLING"]
    assert timeout["observed_count"] == 1
    assert timeout["max_age_us"] == 122000
    assert timeout["firmware_count_max"] == 7

    interrupts = status["sensor_interrupts"]
    assert interrupts["samples"] == 1
    assert interrupts["levels"] == [1, 0, 1, 0, 1, 1]
    assert interrupts["transitions"] == [1029, 1028, 1029, 0, 1029, 0]


def test_pixart_reader_ignores_unknown_oob_without_growth():
    reader = PixartJ3Reader()
    assert not reader._process_oob_line("unrecognized debug text")
    assert reader.status["diagnostic_sequence"] == 0
    assert "diagnostic_messages" not in reader.status
