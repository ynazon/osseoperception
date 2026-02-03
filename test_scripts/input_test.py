#!/usr/bin/env python3 

# ------------------------

import inputs
def main():
    print("Press 'ESC' to quit.")
    while True:
        events = inputs.get_key()
        for event in events:
            if event.ev_type == 'Key':
                if event.state == 1:  # Key pressed
                    print(f'Key {event.code} pressed')
                    if event.code == 'KEY_ESC' or event.code == 'KEY_S':
                        print("Exiting...")
                        return
                elif event.state == 0:  # Key released
                    print(f'Key {event.code} released')

if __name__ == "__main__":
    main() 

# # v1 of testing 's' input code
# print("Press 'ESC' to quit.")
# test = ''
# # while True:
# while test != 'KEY_S':
#     events = inputs.get_key()
#     for event in events:
#         if event.ev_type == 'Key':
#             if event.state == 1:  # Key pressed
#                 print(f'Key {event.code} pressed')
#                 test = event.code
#                 # print(event.code)
#                 if event.code == 'KEY_ESC':
#                     print("Exiting...")
#                     # return
#             # elif event.state == 0:  # Key released
#             #     print(f'Key {event.code} released')
            


# ------------------------
# from pynput import keyboard

# def on_press(key):
#     try:
#         print(f'Alphanumeric key {key.char} pressed')
#     except AttributeError:
#         print(f'Special key {key} pressed')

# def on_release(key):
#     print(f'Key {key} released')
#     if key == keyboard.Key.esc:
#         # Stop listener
#         return False    
    
# # Collect events until released
# with keyboard.Listener(
#         on_press=on_press,
#         on_release=on_release) as listener:
#     listener.join() 

# ------------------------    
# import sshkeyboard

# def on_key_press(key):
#     print(f'Key {key} pressed') 
# def on_key_release(key):
#     print(f'Key {key} released')
#     if key == 'esc':
#         return False  # Stop listener  
     
# sshkeyboard.listen_keyboard(on_press=on_key_press, on_release=on_key_release)

# ------------------------
# import readchar

# print("Press 'q' to quit.")
# while True:
#     key = readchar.readkey()
#     print(f'You pressed: {key}')
#     if key == 'q':
#         print("Exiting...")
#         break           

# ------------------------
# import keyboard
# def on_key_event(event):
#     print(f'Key {event.name} {"pressed" if event.event_type == "down" else "released"}')
#     if event.name == 'esc' and event.event_type == 'down':
#         print("Exiting...")
#         return False  # Stop listener
# keyboard.hook(on_key_event)
# keyboard.wait('esc')  # Keep the program running until 'esc' is pressed 
 

