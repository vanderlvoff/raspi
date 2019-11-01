import pyttsx3

class Vocalizer:

    def speak(self, text):
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()