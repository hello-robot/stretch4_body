
#!/usr/bin/env python3
import time
import math
import sys
import os

# Ensure we can import the module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'stretch4_body/core')))

from stretch4_body.core.rerun_dynamic_plotter import DynamicRerunPlotter

def main():
    print("Starting DynamicRerunPlotter test...")
    plotter = DynamicRerunPlotter("DynamicPlotterTest")
    
    start_time = time.time()
    
    # Simulate a robot status loop
    print("Logging data for 5 seconds...")
    try:
        while time.time() - start_time < 115.0:
            elapsed = time.time() - start_time
            
            # Create a dummy status dictionary with nested structure
            status = {
                'power_periph': {
                    'voltage': 12.0 + math.sin(elapsed),
                    'current': 2.0 + 0.5 * math.cos(elapsed),
                    'board_info': { # Nested non-scalar, should be ignored or traversed
                        'version': '1.0'
                    }
                },
                'lift': {
                    'pos': 0.5 + 0.1 * math.sin(elapsed * 2),
                    'vel': 0.1 * math.cos(elapsed * 2),
                    'motor': {
                         'temp': 35.0 + elapsed
                    }
                },
                'arm': {
                    'pos': 0.1 * elapsed
                },
                'timestamp': time.time(),
                'mode': 'idle' # String, should be ignored
            }
            
            plotter.step(status)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    
    print("Test finished.")

if __name__ == "__main__":
    main()
