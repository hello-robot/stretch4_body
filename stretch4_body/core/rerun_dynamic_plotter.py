
import rerun as rr
import socket
import time

class DynamicRerunPlotter:
    """
    A class that recursively plots scalar values from a dictionary to Rerun.
    """
    def __init__(self, name, open_browser=False, web_port=9090, grpc_port=9877, server_memory_limit="4GB"):
        self.name = name
        self.web_port = web_port
        self.grpc_port = grpc_port
        self._local_ip = self.get_local_ip()
        
        self.web_url = f"http://{self._local_ip}:{web_port}/?url=rerun%2Bhttp://{self._local_ip}:{grpc_port}/proxy"
        if self._local_ip is not None:
             print('=' * 50)
             print(f"Rerun plotter running at {self.web_url}")
             print('=' * 50)

        rr.init(name)
        server_uri = rr.serve_grpc(grpc_port=grpc_port, server_memory_limit=server_memory_limit)
        rr.serve_web_viewer(web_port=web_port, open_browser=open_browser, connect_to=server_uri)

    def get_local_ip(self):
        """
        Get the local IP address of the system that can be accessed by remote computers.
        Returns:
            str: Local IP address.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Connect to an external address to determine the local IP
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        finally:
            s.close()
        return local_ip

    def step(self, status):
        """
        Log the status dictionary to Rerun recursively.
        Args:
            status (dict): The dictionary containing data to log.
        """
        self._recursive_log("", status)

    def _recursive_log(self, path, data):
        for key, value in data.items():
            current_path = f"{path}/{key}" if path else key
            
            if isinstance(value, (int, float)):
                 # Log scalar values
                 rr.log(f"{self.name}/{current_path}", rr.Scalars(value))
            elif isinstance(value, dict):
                # Recurse into sub-dictionaries
                self._recursive_log(current_path, value)
            # Recursion stops for non-dict, non-scalar types (lists, strings, objects, etc.)
