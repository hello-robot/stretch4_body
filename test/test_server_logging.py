import unittest
import os
import tempfile
import logging
from logging.handlers import RotatingFileHandler
from unittest.mock import patch

# Important: these must be imported after HELLO_FLEET_PATH is set
from stretch4_body.tools import stretch_body_server as sbs

class TestServerLogging(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.log_file = os.path.join(self.temp_dir, "test_sbs.log")
        
    def tearDown(self):
        # Clean up files
        for f in os.listdir(self.temp_dir):
            try:
                os.remove(os.path.join(self.temp_dir, f))
            except:
                pass
        os.rmdir(self.temp_dir)

    def test_rotating_file_handler_rotation(self):
        """
        Test that RotatingFileHandler functions correctly, rotating logs
        after reaching 'maxBytes' config, mirroring the stretch_body config.
        """
        handler = RotatingFileHandler(self.log_file, maxBytes=100, backupCount=2)
        logger = logging.getLogger("test_rot_logger")
        logger.setLevel(logging.DEBUG)
        
        # Make sure no other handlers conflict
        logger.handlers = []
        logger.addHandler(handler)
        
        # Use a flat formatter to easily count bytes
        # Each message will be 10 characters: "M: XXXXXXX"
        formatter = logging.Formatter('%(message)s')
        handler.setFormatter(formatter)
        
        # 10 chars + 1 newline = 11 bytes per message
        # We need 10 messages to reach 110 > 100 bytes (triggers rotation)
        for i in range(12):
            logger.info(f"M: {i:07d}")
            
        # Verify file exists
        self.assertTrue(os.path.exists(self.log_file))
        # The first backup should be created due to maxBytes=100
        self.assertTrue(os.path.exists(self.log_file + ".1"))
        
        with open(self.log_file, "r") as f:
            lines = f.readlines()
            self.assertTrue(any("M: 0000010" in line for line in lines))
            
        with open(self.log_file + ".1", "r") as f:
            lines_backup = f.readlines()
            self.assertTrue(any("M: 0000000" in line for line in lines_backup))
            
        # Manually close handler
        handler.close()
        logger.removeHandler(handler)

    def test_tail_log_file_scenario_1_rename(self):
        """
        Scenario 1: The log file is renamed (e.g., test.log -> test.log.1) 
        and a new blank file is created.
        """
        # Create initial file
        with open(self.log_file, "w") as f:
            f.write("startup line\n")
            
        sleep_ticks = 0
        read_lines = []
        
        def mock_sleep(secs):
            nonlocal sleep_ticks
            sleep_ticks += 1
            if sleep_ticks == 1:
                # Write to the file, it should be read
                with open(self.log_file, "a") as fd:
                    fd.write("line 2\n")
            elif sleep_ticks == 2:
                # Scenario 1: Rename to .1 and create new target file
                os.rename(self.log_file, self.log_file + ".1")
                with open(self.log_file, "w") as fd:
                    fd.write("line 3\n")
            elif sleep_ticks == 3:
                # Exit
                raise KeyboardInterrupt()

        def mock_color_print(line):
            read_lines.append(line.strip())
            
        with patch("stretch4_body.tools.stretch_body_server.time.sleep", side_effect=mock_sleep), \
             patch("stretch4_body.tools.stretch_body_server.color_print", side_effect=mock_color_print):
            sbs.tail_log_file(self.log_file)
            
        self.assertIn("line 2", read_lines)
        self.assertIn("line 3", read_lines)

    def test_tail_log_file_scenario_2_truncate(self):
        """
        Scenario 2: The log file gets truncated, resetting file size but keeping same inode.
        """
        with open(self.log_file, "w") as f:
            f.write("startup line 1\n")
            f.write("startup line 2\n")

        sleep_ticks = 0
        read_lines = []
        
        def mock_sleep(secs):
            nonlocal sleep_ticks
            sleep_ticks += 1
            if sleep_ticks == 1:
                # Truncate the file and write to it
                with open(self.log_file, "w") as fd:
                    fd.truncate(0)
                    fd.write("line A\n")
            elif sleep_ticks == 2:
                raise KeyboardInterrupt()
                
        def mock_color_print(line):
            read_lines.append(line.strip())
            
        with patch("stretch4_body.tools.stretch_body_server.time.sleep", side_effect=mock_sleep), \
             patch("stretch4_body.tools.stretch_body_server.color_print", side_effect=mock_color_print):
            sbs.tail_log_file(self.log_file)

        self.assertIn("line A", read_lines)

if __name__ == "__main__":
    unittest.main()
