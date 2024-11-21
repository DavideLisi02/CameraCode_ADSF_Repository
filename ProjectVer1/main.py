import settings
import threading

# Event to signal thread termination (must be defined here to be accessible to both main and settings)
stop_event = threading.Event()

def main():
    settings_dialog = settings.SettingsDialog() # Pass stop_event
    settings_dialog.protocol("WM_DELETE_WINDOW")
    settings_dialog.mainloop()

if __name__ == "__main__":
    main()
