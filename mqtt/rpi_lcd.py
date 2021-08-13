from RPLCD import i2c, CharLCD, cleared, cursor
import time, subprocess, netifaces

lcd = i2c.CharLCD(i2c_expander="PCF8574", address=0x3f, auto_linebreaks=False)
ssid_tc = ""
ip_tc = ""



long_string = "Really long string to show on such a short display"
ip_info = "192.168.1.1"
framebuffer = [
    'SSID name',
    'IP address',
    ]

def get_wifi_info():
    ssid = subprocess.check_output(['sudo', 'iwgetid'])
    ssid_name = ssid.decode("utf-8").split('"')[1]
    
    interface_data = netifaces.ifaddresses('wlan0')
    ip_info = interface_data[netifaces.AF_INET][0]['addr']
    return (ssid_name, ip_info)
    
    
def write_to_lcd(lcd, framebuffer, num_cols):
    lcd.home()
    for row in framebuffer:
        lcd.write_string(row.ljust(num_cols)[:num_cols])
        lcd.write_string('\r\n')

def loop_string(lcd, framebuffer, row, num_cols, delay=0.2):
    padding = ' ' * num_cols
    s1 = padding + ssid_info + padding
    s2 = padding + ip_info + padding
    longstr = s1
    if len(s1) < len(s2):
        longstr = s2

    for i in range(len(longstr) - num_cols + 1):
        framebuffer[0] = s1[i:i+num_cols]
        framebuffer[1] = s2[i:i+num_cols]
        
        write_to_lcd(lcd, framebuffer, num_cols)
        time.sleep(delay)


while True:
    try:
        (ssid_info, ip_info) = get_wifi_info()
    except:
        ssid_info = "Not connected"
        ip_info = "Unknown"
    
    if ssid_tc != ssid_info or ip_tc != ip_info:
        lcd.close(clear=True)
        lcd = i2c.CharLCD(i2c_expander="PCF8574", address=0x3f, auto_linebreaks=False)
        
    
    if len(ssid_info) > 16 or len(ip_info) > 16:
        loop_string(lcd, framebuffer, 0 ,16)
    else:
        lcd.write_string(f"{ssid_info}\r\n{ip_info}")
    
    time.sleep(5)