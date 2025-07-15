from gpiozero import OutputDevice
from time import sleep

def main():
    print('Hi from pkg_testmagnet_py.')
    pin_num = 24  # BCM pin number, not BOARD number!

    # Create an OutputDevice for the electromagnet
    magnet = OutputDevice(pin_num)

    # Turn magnet on
    magnet.on()
    print("Magnet is ON!")
    sleep(5)

    # Turn magnet off
    magnet.off()
    print("Magnet is OFF!")

if __name__ == '__main__':
    main()
