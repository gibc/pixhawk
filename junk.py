from pynput import keyboard

def on_press(key):
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['1', '2', 'left', 'right']:  # keys of interest
        # self.keys.append(k)  # store it in global-like variable 
        print("***************************************************")
        print("\033c", end="")
        print('Key pressed: ' + k)    
        print("***************************************************")
        quit()
        return False  # stop listener; remove this if want more keys

listener = keyboard.Listener(on_press=on_press)
listener.start()  # start to listen on a separate thread123333321