import os
import psutil


def get_cpu_temp():
    """ Return CPU temperature """
    cpu_temp = 0
    mypath = "/sys/class/thermal/thermal_zone0/temp"
    with open(mypath, 'r') as f:
        for line in f:
            cpu_temp = line

    cpu_temp = float(cpu_temp)/1000
    cpu_temp = round(cpu_temp, 1)
    return str(cpu_temp)


def get_gpu_temp():
    """ Return GPU temperature as a character string"""
    gpu_temp = os.popen('/opt/vc/bin/vcgencmd measure_temp').readline()
    return gpu_temp.replace("temp=", "")


def get_cpu_use():
    """ Return CPU usage using psutil"""
    cpu_percent = psutil.cpu_percent()
    return str(cpu_percent)


def get_ram_info():
    """ Return RAM usage using psutil """
    virtual_memory = psutil.virtual_memory()[2]
    return str(virtual_memory)


def get_swap_info():
    """ Return swap memory  usage using psutil """
    swap_memory = psutil.swap_memory()[3]
    return str(swap_memory)
