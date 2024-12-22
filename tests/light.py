"""
This script initializes and controls an LED strip using the rpi_ws281x library.
It sets all LEDs to red color and displays the result.
"""

import logging
import rpi_ws281x

# Initialize logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# LED configuration
LED_COUNT = 16  # Number of LED pixels
LED_PIN = 12    # GPIO pin connected to the pixels (must support PWM)


def main():
    logging.info(f"rpi_ws281x version: {rpi_ws281x.__version__}")

    # Create NeoPixel object with appropriate configuration
    strip = rpi_ws281x.Adafruit_NeoPixel(LED_COUNT, LED_PIN)
    strip.begin()

    # Set all pixels to red
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, rpi_ws281x.Color(255, 0, 0))  # Red color

    # Update the LED strip to show the changes
    strip.show()
    logging.info("LED strip updated with red color.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logging.info("Script terminated by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
