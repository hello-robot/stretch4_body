"""ServerStatusReader: subscribes to the server status socket and sends nothing."""
import zmq

from stretch4_body.core import server_status_reader as ssr


def test_no_status_socket_means_no_server(tmp_path):
    reader = ssr.ServerStatusReader(port=str(tmp_path / 'port_status'))
    assert reader.open() is False
    assert reader.read(timeout=0.01) is None
    reader.close()


def test_reads_the_latest_status_from_a_publisher(tmp_path):
    port = str(tmp_path / 'port_status')
    pub = zmq.Context.instance().socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind('ipc://%s' % port)
    try:
        with ssr.ServerStatusReader(port=port) as reader:
            got = None
            for i in range(200):       # a SUB misses messages sent before it joined
                pub.send_pyobj({'server': {'status_id': i}})
                got = reader.read(timeout=0.02)
                if got is not None:
                    break
            assert got is not None and 'server' in got
            assert reader.socket.getsockopt(zmq.TYPE) == zmq.SUB
        assert reader.socket is None
    finally:
        pub.close(linger=0)


def test_silent_server_reads_none(tmp_path):
    port = str(tmp_path / 'port_status')
    pub = zmq.Context.instance().socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind('ipc://%s' % port)
    try:
        with ssr.ServerStatusReader(port=port) as reader:
            assert reader.read(timeout=0.05) is None
    finally:
        pub.close(linger=0)
