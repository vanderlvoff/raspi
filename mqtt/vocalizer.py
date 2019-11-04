#import pyttsx3

#class Vocalizer:

    #def __init__(self):
    #    #Nothing
    #    print("ok")
        
#    def speak(self, text):
#        engine = pyttsx3.init(driverName='espeak')
#        engine.say(text)
#        engine.runAndWait()
        
#voc = Vocalizer()
#voc.speak("Hello world!")

from espeak import espeak

espeak.synth("Hello")