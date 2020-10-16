# -*- coding: utf-8 -*-
import re
import time
import math
from flir_ptu.stream import Stream
from vision_utils.logger import get_logger
logger =  get_logger()

"""
The below are the config definitions that are used to autogenerate and add
commands to the ptu class.

The syntax for adding a new command to the ptu class is
    name_of_command(this will be the name of the function): {
        If you need a get, use the below.
        "get" : [command_used_to_read_value, r"regex"],
        Note:
            The regex used is expected to return a single name
            parameter named `expected` which will be the returned
            value.

        If you need a set, use the below.
        "set" : [function_send, wait, function_wait]
        Note:
            `function_send` is passed the values that are passed to function
            `self.name_of_command(args)` and it needs to return the command that
            should be sent.
            `wait` is boolean that is used to determine whether you need the function
            to block.
            `function_wait` is passed the class as the first argument and the args as
            the rest. The function should return the value that is used in conjuction with
            the get command to determine whether the ptu has finished the operation.
    }
"""

cmds = {
    "wait": {
        "set": [lambda pos: "a", False, False],
        "get": ["a",
                r"(?P<expected>.*)\r\n"
                ]
    },
    "reset": {
        "set": [lambda pos: "r", False, False],
        "get": ["r",
                r"(?P<expected>.*)\r\n"
                ]
    },
    "pan": {
        "set": [lambda pos: "pp" + str(pos), False, False],
        "get": ["pp",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "pan_speed": {
        "set": [lambda pos: "ps" + str(pos), False, False],
        "get": ["ps",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "tilt_speed": {
        "set": [lambda pos: "ts" + str(pos), False, False],
        "get": ["ts",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "tilt_speed_max": {
        "set": [lambda pos: "tu" + str(pos), False, False],
        "get": ["tu",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "pan_speed_max": {
        "set": [lambda pos: "pu" + str(pos), False, False],
        "get": ["pu",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "pan_accel": {
        "set": [lambda pos: "pa" + str(pos), False, False],
        "get": ["pa",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "tilt": {
        "set": [lambda pos: "tp" + str(pos), False, False],
        "get": ["tp",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "pan_offset": {
        "set": [lambda pos: "po" + str(pos),
                False,
                lambda self, pos: int(self.pan()) + pos
                ],
        "get": ["po",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "tilt_offset": {
        "set": [lambda pos: "to" + str(pos), False, lambda self, pos: int(self.tilt()) + pos],
        "get": ["to",
                r"\s*(?P<expected>-*?\d+)\r\n"
                ]
    },
    "set_speed_mode": {
        "set": [lambda pos: "cv", False, False],
        "get": ["cv",
                r"\s*(?P<expected>.*)\r\n"
                ]
    },
    "set_position_mode": {
        "set": [lambda pos: "df", False, False],
        "get": ["df",
                r"\s*(?P<expected>.*)\r\n"
                ]
    }
}


def position_decorator(cls):
    def generate_functions(cls, item, key):
        if item.get("get"):
            getter_valid = True
            read_string, regex = item["get"]
        else:
            getter_valid = False

        if item.get("set"):
            send_string, wait_completion, value_mod_func = item["set"]

        def template(self, *args):
            if len(args):
                cmd = send_string(*args)
                # logger.info("Send command: {}".format(cmd))
                self.send_command(send_string(*args))
                if wait_completion:
                    func = getattr(self, key)
                    if value_mod_func:
                        checking_value = value_mod_func(self, *args)
                    else:
                        checking_value = args[0]
                    logger.info("Blocking to get value: {}".format(checking_value))
                    while True:
                        value = func()
                        if int(value) != checking_value:
                            time.sleep(.1)
                        else:
                            logger.info("Value read: {}".format(value))
                            break
            else:
                if getter_valid:
                    return self.read_command(read_string, regex)
                else:
                    logger.warning("No valid getter command for {}".format(key))

        setattr(cls, key, template)

    for key in cmds:
        item = cmds[key]
        generate_functions(cls, item, key)
    return cls


@position_decorator
class PTU:

    def __init__(self, host, port, debug=False):
        self.stream = Stream(host, port, testing=debug)

    def connect(self):
        self.stream.connect()

    def send_command(self, command):
        self.stream.send(command)
        self.stream.read_until("\n")

    def read_command(self, command, regex):
        self.stream.send(command)
        data = self.stream.read_until("*").decode()
        # print(data)
        data = self.stream.read_until("\n").decode()
        match = re.match(regex, data)
        # print(data)
        if match:
            return match.group("expected")
        else:
            logger.error("Error parsing regex of")

    def pan_angle(self, angle_value=False):
        if type(angle_value) !=  bool:
            self.pan(math.ceil(angle_value/(92.5714/3600)))
        else:
            data = self.pan()
            return round(float(data) * (92.5714/3600), 2)

    def tilt_angle(self, angle_value=False):
        if type(angle_value) != bool:
            self.tilt(math.ceil(angle_value/(46.2857/3600)))
        else:
            data = self.tilt()
            return round(float(data) * (46.2857/3600), 2)
