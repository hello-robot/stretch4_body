import os
import time
import tempfile
import unittest
from unittest.mock import MagicMock, patch
import subprocess

from stretch4_body.core.hello_utils import play_sound, _sounds_playing, get_sounds_dir


class TestPlaySound(unittest.TestCase):

    def setUp(self):
        _sounds_playing.clear()
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.sound1 = os.path.join(self.tmp_dir.name, "sound1.wav")
        self.sound2 = os.path.join(self.tmp_dir.name, "sound2.wav")
        with open(self.sound1, "w") as f:
            f.write("dummy audio content 1")
        with open(self.sound2, "w") as f:
            f.write("dummy audio content 2")

    def tearDown(self):
        _sounds_playing.clear()
        self.tmp_dir.cleanup()

    @patch("subprocess.Popen")
    def test_play_sound_file_not_found(self, mock_popen):
        play_sound("/non/existent/file.wav")
        mock_popen.assert_not_called()
        self.assertEqual(len(_sounds_playing), 0)

    @patch("subprocess.Popen")
    def test_play_sound_new_file(self, mock_popen):
        mock_proc = MagicMock()
        mock_proc.poll.return_value = None  # Running
        mock_popen.return_value = mock_proc

        play_sound(self.sound1)

        mock_popen.assert_called_once_with(["aplay", self.sound1], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        abs_sound1 = os.path.abspath(self.sound1)
        self.assertIn(abs_sound1, _sounds_playing)
        self.assertEqual(_sounds_playing[abs_sound1], mock_proc)

    @patch("subprocess.Popen")
    def test_play_sound_already_playing_does_not_replay(self, mock_popen):
        mock_proc1 = MagicMock()
        mock_proc1.poll.return_value = None  # Running
        mock_popen.return_value = mock_proc1

        play_sound(self.sound1)
        self.assertEqual(mock_popen.call_count, 1)

        # Try to play sound1 again while it's still running
        play_sound(self.sound1)
        self.assertEqual(mock_popen.call_count, 1)  # Should not have called Popen again

    @patch("subprocess.Popen")
    def test_play_sound_stop_current_playing_true(self, mock_popen):
        mock_proc1 = MagicMock()
        mock_proc1.poll.return_value = None  # Running
        mock_proc2 = MagicMock()
        mock_proc2.poll.return_value = None  # Running

        mock_popen.side_effect = [mock_proc1, mock_proc2]

        play_sound(self.sound1)
        self.assertIn(os.path.abspath(self.sound1), _sounds_playing)

        # Play sound2 with stop_current_playing=True (default)
        play_sound(self.sound2, stop_current_playing=True)

        mock_proc1.terminate.assert_called_once()
        self.assertNotIn(os.path.abspath(self.sound1), _sounds_playing)
        self.assertIn(os.path.abspath(self.sound2), _sounds_playing)

    @patch("subprocess.Popen")
    def test_play_sound_stop_current_playing_false(self, mock_popen):
        mock_proc1 = MagicMock()
        mock_proc1.poll.return_value = None  # Running
        mock_proc2 = MagicMock()
        mock_proc2.poll.return_value = None  # Running

        mock_popen.side_effect = [mock_proc1, mock_proc2]

        play_sound(self.sound1)
        play_sound(self.sound2, stop_current_playing=False)

        mock_proc1.terminate.assert_not_called()
        self.assertIn(os.path.abspath(self.sound1), _sounds_playing)
        self.assertIn(os.path.abspath(self.sound2), _sounds_playing)

    @patch("subprocess.Popen")
    def test_cleanup_finished_sounds(self, mock_popen):
        mock_proc1 = MagicMock()
        mock_proc1.poll.return_value = 0  # Finished
        mock_popen.return_value = mock_proc1

        abs_sound1 = os.path.abspath(self.sound1)
        _sounds_playing[abs_sound1] = mock_proc1

        mock_proc2 = MagicMock()
        mock_proc2.poll.return_value = None  # Running
        mock_popen.return_value = mock_proc2

        # Calling play_sound on sound2 should clean up sound1 because sound1 is finished
        play_sound(self.sound2)

        self.assertNotIn(abs_sound1, _sounds_playing)
        self.assertIn(os.path.abspath(self.sound2), _sounds_playing)


if __name__ == "__main__":
    unittest.main()
