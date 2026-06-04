import os
import subprocess
import threading
import queue
import speech_recognition as sr

# Function to start GStreamer pipeline for audio capture
def start_gstreamer_pipeline(q):
    # GStreamer pipeline command to capture audio from microphone
    gst_command = [
        'gst-launch-1.0',
        'alsasrc', '!', 
        'audioconvert', '!', 
        'audioresample', '!', 
        'audio/x-raw,format=S16LE,channels=1,rate=48000', '!',
        'queue', '!',
        'fdsink', 'fd=1'
    ]

    # Run GStreamer pipeline and capture stdout
    gst_process = subprocess.Popen(gst_command, stdout=subprocess.PIPE, bufsize=1024)
    
    while True:
        data = gst_process.stdout.read(1024)
        if data:
            q.put(data)
        else:
            break

# Function to recognize speech from audio data
def recognize_speech(q):
    recognizer = sr.Recognizer()
    command_list = ["forward", "backward", "left", "right", "stop"]
    while True:
        try:
            audio_data = q.get()
            if not audio_data:
                continue

            # Convert byte data to AudioData
            audio = sr.AudioData(audio_data, sample_rate=48000, sample_width=2)
            command = recognizer.recognize_google(audio).lower()
            print(f"Transcript: {command}")

            for cmd in command_list:
                if cmd in command:
                    print(cmd)

            #if "forward" in command:
            #    print("I go forward")

        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")

if __name__ == "__main__":
    audio_queue = queue.Queue()

    # Start GStreamer pipeline in a separate thread
    gstreamer_thread = threading.Thread(target=start_gstreamer_pipeline, args=(audio_queue,))
    gstreamer_thread.daemon = True
    gstreamer_thread.start()

    # Start speech recognition in the main thread
    recognize_speech(audio_queue)
