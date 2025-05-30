from os import environ
from dotenv import load_dotenv
from websocket import WebSocketApp
from threading import Thread, Event
from pyaudio import PyAudio, paFloat32
from time import time, sleep
import prompt as prompt
import base64
import struct
import numpy
import requests
import json
import re

# -------------------- variables --------------------

# .env
load_dotenv()
OPENAI_API_KEY = environ.get("OPENAI_API_KEY")
NAVEL_IP = environ.get("NAVEL_IP")

# Websocket configuration
URL = "wss://api.openai.com/v1/realtime?model=gpt-4o-mini-realtime-preview-2024-12-17"
HEADERS = [
    f"Authorization: Bearer {OPENAI_API_KEY}",
    "OpenAI-Beta: realtime=v1",  
]

# Audio configuration
CHUNK = 1024        # Number of audio frames per buffer
FORMAT = paFloat32  # Audio format
CHANNELS = 1        # Mono audio
RATE = 24000        # Sample rate (Hz)

# Define pyaudio and websockets
audio = None
ws = None

# Global variables to manage the input thread and shutdown event
input_thread = None
navel_thread = None
emotion_thread = None
stop_event = Event()

# -------------------- helper functions --------------------

def get_current_emotion():
    res = requests.get(f"http://{NAVEL_IP}:8000/emotion")
    msg = json.loads(res.content)
    return msg['emotion']

def split_json_and_text(input_str):
    # Match the first JSON object at the start of the string
    match = re.match(r'(\{.*?\})(.*)', input_str.strip(), re.DOTALL)
    
    if match:
        json_part = match.group(1)
        text_part = match.group(2).strip()
        
        try:
            json_obj = json.loads(json_part)
            return json_obj, text_part
        except json.JSONDecodeError:
            raise ValueError("Could not decode JSON from the input string.")
    else:
        raise ValueError("No JSON object found at the beginning of the string.")
    
def float_to_16bit_pcm(float32_array):
    clipped = [max(-1.0, min(1.0, x)) for x in float32_array]
    pcm16 = b''.join(struct.pack('<h', int(x * 32767)) for x in clipped)
    return pcm16

def base64_encode_audio(float32_array):
    pcm_bytes = float_to_16bit_pcm(float32_array)
    encoded = base64.b64encode(pcm_bytes).decode('ascii')
    return encoded

# -------------------- thread functions --------------------

def send_navel(text):
    global input_thread
    print(f"[Text] sending '{text}' to navel")
    url = f"http://{NAVEL_IP}:8000/say"
    payload = { "message": text }
    headers = { "Content-Type": "application/json" }
    # blocking request. only continues after navel has finished speaking
    response = requests.post(url, headers=headers, json=payload)

    # Start the input thread if it is not already running.
    if input_thread is None or not input_thread.is_alive():
        stop_event.clear()
        input_thread = Thread(
            target=send_audio, args=(ws, stop_event), daemon=True
        )
        input_thread.start()

def send_emotion(emotion, duration):
    print(f"[Emotion] Sending '{emotion}' to Navel for {duration}ms")

    payload = { "emotion": emotion }
    headers = { "Content-Type": "application/json" }
    requests.post(f"http://{NAVEL_IP}:8000/emotion", headers=headers, json=payload)
    sleep((duration) / 1000)
    requests.post(f"http://{NAVEL_IP}:8000/reset/face")

def send_audio(ws, stop_event):
    # Audio-Stream starten
    stream = audio.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print("[Audio] stream started.")
    last_emotion = "None"
    emotion_timer = 0
    try:
        while not stop_event.is_set():
            # Update emotion every 2 seconds if it changes
            if time() - emotion_timer > 1.0:
                current_emotion = get_current_emotion()
                if current_emotion != last_emotion:
                    print(f"[Emotion] Aktualisiert: {current_emotion}")
                    event = {
                        "type": "conversation.item.create",
                        "previous_item_id": None,
                        "item": {
                            "type": "message",
                            "role": "user",
                            "content": [
                                {
                                    "type": "input_text",
                                    "text": f"Der Nutzer scheint jetzt {current_emotion} zu sein"
                                }
                            ]
                        }
                    }
                    ws.send(json.dumps(event))
                    
                    last_emotion = current_emotion
                emotion_timer = time()
            
            # Continuously capture microphone audio and send it to the websocket.
            data = stream.read(CHUNK, exception_on_overflow=False)
            float_data = numpy.frombuffer(data, dtype=numpy.float32)
            # Create an event to send the audio.
            event = {
                "type": "input_audio_buffer.append",
                "audio": base64_encode_audio(float_data)
            }
            ws.send(json.dumps(event))
    finally:
        stream.stop_stream()
        stream.close()
        print("[Audio] stream stopped.")

def watch_for_stop(ws, stop_event):
    # Block until the server stop_event is set, then close the ws connection
    stop_event.wait()
    ws.close()

# -------------------- websocket functions --------------------

def on_open(ws):
    global input_thread, stop_event
    print("[Socket] Connected to server.")
    # record audio
    if input_thread is None or not input_thread.is_alive():
        stop_event.clear()
        input_thread = Thread(
            target=send_audio, args=(ws, stop_event), daemon=True
        )
        input_thread.start()

def on_message(ws, message):
    global navel_thread, emotion_thread, stop_event
    data = json.loads(message)
    message_type = data['type']
    print(f"[Socket] received: {message_type}")

    match message_type:
        case "error":
            print(f"[Socket] error: {data}")

        case "response.output_item.done":
            print("[Socket] stopping audio stream")
        
            # Signal the input thread to stop.
            stop_event.set()
            response = data['item']['content'][0]['text']
            obj, text = split_json_and_text(response)
            # send emotion
            if emotion_thread is None or not emotion_thread.is_alive():
                emotion_thread = Thread(
                    target=send_emotion, args=(obj['emotion'], obj['duration']), daemon=True
                )
                emotion_thread.start()  
            # send text
            if navel_thread is None or not navel_thread.is_alive():
                navel_thread = Thread(
                    target=send_navel, args=(text,), daemon=True
                )
                navel_thread.start()

        case "session.created":
            # update session
            event = {
                "type": "session.update",
                "session": {
                    "modalities": ["text"],
                    "instructions": prompt.text,
                    "turn_detection": {
                        "type": "server_vad",
                        "threshold": 0.7,
                        "prefix_padding_ms": 300,
                        "silence_duration_ms": 800,
                        "create_response": True
                    },
                    "input_audio_noise_reduction": {
                        "type": "far_field"
                    },
                }
            }
            ws.send(json.dumps(event))

# ------------------- entrypoint -------------------

def main(stop_event=Event()):
    global audio, ws
    audio = PyAudio()
    ws = WebSocketApp(
        URL,
        header=HEADERS,
        on_open=on_open,
        on_message=on_message,
    )
    Thread(
        target=watch_for_stop, args=(ws, stop_event), daemon=True
    ).start()
    ws.run_forever()

if __name__ == "__main__":
    main()