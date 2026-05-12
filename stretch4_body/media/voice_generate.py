#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys
from gtts import gTTS
from pydub import AudioSegment


def list_languages():
    """Prints common language codes for gTTS."""
    # gTTS doesn't 'list' local voices like pyttsx3, it uses language locales.
    langs = {
        "en (US)": "en",
        "en (UK)": "en-uk",
        "en (AU)": "en-au",
        "fr (France)": "fr",
        "es (Spain)": "es",
        "de (Germany)": "de"
    }
    print(f"{'Language':<15} | {'Code'}")
    print("-" * 25)
    for name, code in langs.items():
        print(f"{name:<15} | {code}")
    print("\nNote: gTTS uses internet-based Google voices.")


def make_wave(text, lang_code, output_file, play_audio):
    """Generates audio using gTTS, converts to WAV, and plays via aplay."""
    temp_mp3 = "temp_speech.mp3"

    try:
        print(f"Requesting audio from Google (lang: {lang_code})...")
        tts = gTTS(text=text, lang=lang_code, slow=False)
        tts.save(temp_mp3)

        # Convert MP3 to WAV for aplay compatibility
        audio = AudioSegment.from_mp3(temp_mp3)
        audio.export(output_file, format="wav")
        os.remove(temp_mp3)  # Clean up temp file

        print(f"Saved to: {output_file}")

        if play_audio:
            print(f"Playing {output_file} via aplay...")
            subprocess.run(['/usr/bin/aplay', output_file])

    except Exception as e:
        print(f"Error: {e}")
        if os.path.exists(temp_mp3):
            os.remove(temp_mp3)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="REx Voice Generator (gTTS + aplay)")
    parser.add_argument("text", nargs="?", help="The phrase to convert to speech")
    parser.add_argument("-v", "--voice", default="en-au", help="Language code (default: en-au)")
    parser.add_argument("-o", "--output", default="output.wav", help="Output filename (default: output.wav)")
    parser.add_argument("-l", "--list", action="store_true", help="List common language codes")
    parser.add_argument("-p", "--play", action="store_true", help="Play the file after generating")

    args = parser.parse_args()

    if args.list:
        list_languages()
    elif args.text:
        make_wave(args.text, args.voice, args.output, args.play)
    else:
        parser.print_help()