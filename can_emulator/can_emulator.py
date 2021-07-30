#!/usr/bin/python3

'''
********************************************************************************
*                             All Rights Reserved
*
*@file      tools/can_enmulator.py
*@brief     AC CAN bus Emulator
*@version   V0.01
*@author    zburid#outlook.com
*@date      2020/07/23
*@attention
*@note
********************************************************************************
'''

from __future__ import print_function

import os
import sys
import time
import math
import json
import queue
import threading
import copy

import can
from can.interface import Bus

from tkinter import *
from tkinter import ttk
from tkinter.filedialog import askopenfilename

class CanBackstage(threading.Thread):
    '''
        Args:
            self.can_interface:
                'pcan'
                'vector'
            self.can_bitrate:
                250000                  250Kbit/s
                500000                  500Kbit/s
                others
    '''
    def __init__(self, cmdqueue, append_list):
        threading.Thread.__init__(self)
        self.cmdqueue = cmdqueue
        self.append_list = append_list
        self.can_interface = None
        self.can_channel = None
        self.can_bitrate = None
        self.bus = None
        self.msg = None
        self.is_trace = False
        self.has_connected = False

    def run(self):
        print("Start CAN backstage process!")
        while True:
            if not self.cmdqueue.empty():
                data = self.cmdqueue.get_nowait()
                if data['cmd'] == 'reset':
                    self.can_interface = data['interface']
                    self.can_channel = data['channel']
                    self.can_bitrate = data['bitrate']
                    self.is_trace = False
                    self.has_connected = False
                elif data['cmd'] == 'shutdown':
                    self.shutdown()
                elif data['cmd'] == 'connect':
                    if self.connect():
                        self.has_connected = True
                elif data['cmd'] == 'start_listen':
                    self.is_trace = True
                    print("Now start listening CAN bus")
                elif data['cmd'] == 'stop_listen':
                    self.is_trace = False
                    print("Now stop listening CAN bus")
                elif data['cmd'] == 'send':
                    self.bus.send(data['msg'])
                    self.append_list(data['msg'], is_rx=False)
                elif data['cmd'] == 'exit':
                    self.shutdown()
                    break
                else:
                    print("Unknow command: ", data)
            else:
                if self.is_trace:
                    self.msg = self.bus.recv(timeout=0.1)
                    if self.msg:
                        self.append_list(self.msg, is_rx=True)
                else:
                    time.sleep(0.5)
        print("Exit CAN backstage process!")

    def connect(self):
        self.shutdown()
        try:
            self.bus = Bus(bustype=self.can_interface,
                            channel=self.can_channel,
                            bitrate=int(self.can_bitrate))
        except Exception as e:
            self.bus = None
            print("Fault: ", e)
            return False
        else:
            return True

    def shutdown(self):
        res = True
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception as e:
                print("Fault: ", e)
                res = False
        self.bus = None
        self.msg = False
        return res

class FieldTable():
    def __init__(self, root, label, canid, field, transmitter):
        assert root != None, "Error: No root tkinter handler!"
        self.label = label
        self.canid = canid
        self.bitrange = field['bitrange']
        self.dtype = field['truevalue']['type']
        if self.dtype == 'cal':
            self.index_min = field['truevalue']['min']
            self.index_max = field['truevalue']['max']
            self.offset = field['truevalue']['offset']
            self.scale = field['truevalue']['scale']
        elif self.dtype == 'lut':
            self.truevalue = copy.copy(field['truevalue'])
            self.truevalue.pop('type')
            self.index_min = 0
            self.index_max = len(self.truevalue) - 1
        else:
            raise ValueError("Unknow field data type")
        self.index_default = field.get('default', self.index_min)
        self.transmitter = transmitter
        self.index = 0
        self.field_value = 0
        self.canvas = Canvas(root, width=290, height=20)
        self.init()

    def init(self):
        self.canvas.create_text((100, 10), text=self.label)
        self.canvas.create_rectangle(206, 2, 280, 20, width=1, outline='gray',
                                     fill=None)
        self.canvas.create_text((201, 10), tags=('opt', 'dec'), text="<")
        self.canvas.create_text((243, 10), tags=('value'), text="")
        self.canvas.create_text((285, 10), tags=('opt', 'add'), text=">")

        self.canvas.tag_bind('opt', '<Button-1>', self.change)
        self.canvas.tag_bind('label', '<Double-1>', self.reset)

        self.reset(True)

    def destroy(self):
        self.canvas.destroy()

    def reset(self, force=False):
        if self.index != self.index_min or force:
            self.index = self.index_default
            self.update()

    def update(self):
        if self.dtype == 'cal':
            text = str((self.index + self.offset) * self.scale)
            self.field_value = self.index
        else:
            key = list(self.truevalue.keys())[self.index]
            text = self.truevalue[key]
            self.field_value = int(key)
        itemtag = ('value')
        self.canvas.itemconfig(itemtag, fill='black', text=text, tags=itemtag)
        self.set_transmitter()

    def change(self, event):
        item = self.canvas.find_closest(event.x, event.y)
        tags = self.canvas.gettags(item)

        if 'dec' in tags:
            if self.index > self.index_min:
                self.index -= 1
            else:
                return
        elif 'add' in tags:
            if self.index < self.index_max:
                self.index += 1
            else:
                return
        else:
            print("Unknow event in field table: ", tags)
        self.update()

    def blink(self):
        return

    def set_transmitter(self):
        self.transmitter[self.canid]['lock'].acquire()
        bitbegin = self.bitrange[0]
        bitlength = self.bitrange[1] - self.bitrange[0] + 1
        tmpval = self.field_value
        while bitlength > 0:
            bits = min(8 - bitbegin % 8, bitlength)
            bitmask = (1 << bits) - 1

            value = (tmpval & bitmask) << (bitbegin % 8)
            tmpval = tmpval >> bits

            bitmask = bitmask << (bitbegin % 8)
            index = bitbegin // 8
            self.transmitter[self.canid]['msg'].data[index] &= ~bitmask
            self.transmitter[self.canid]['msg'].data[index] |= value

            bitlength -= bits
            bitbegin += bits
        self.transmitter[self.canid]['lock'].release()

class CanTrace():
    def __init__(self, root, is_waterfall_mode=False, is_rx=False, maxsize=100):
        assert root != None, "Error: No root tkinter handler!"
        self.is_waterfall_mode = is_waterfall_mode
        self.is_rx = is_rx
        self.maxsize = maxsize
        self.style = ttk.Style()
        self.trace_head = ("0", "1", "2", "3", "4", "5", "6")
        self.tvw_trace = ttk.Treeview(root, height=10,
                                      show="headings", columns=self.trace_head)
        self.canlist = {}

    def init(self, row, col, rowspan=4, colspan=8, padx=5, pady=5):
        self.style.configure("Treeview", font=('Consolas', 10))
        self.tvw_trace.grid(row=row, column=col,
                    rowspan=rowspan, columnspan=colspan, padx=padx, pady=pady)
        self.tvw_trace.column("0", width=100, anchor='e')
        self.tvw_trace.column("1", width=35, anchor='center')
        self.tvw_trace.column("2", width=40, anchor='center')
        self.tvw_trace.column("3", width=35, anchor='center')
        self.tvw_trace.column("4", width=90, anchor='center')
        self.tvw_trace.column("5", width=35, anchor='center')
        self.tvw_trace.column("6", width=265, anchor='w')
        if self.is_waterfall_mode:
            self.tvw_trace.heading("0", text='timestamp')
        else:
            self.tvw_trace.heading("0", text='counter')
        self.tvw_trace.heading("1", text='bus')
        self.tvw_trace.heading("2", text='dir')
        self.tvw_trace.heading("3", text='rff')
        self.tvw_trace.heading("4", text='can id')
        self.tvw_trace.heading("5", text='dlc')
        self.tvw_trace.heading("6", text='data bytes')
        self.tvw_trace.tag_configure('errframe', background='orange',
                                     foreground='red')
    def clear(self):
        items = self.tvw_trace.get_children()
        for item in items:
            self.tvw_trace.delete(item)

    def update(self, ts, msg):
        trace_data = (msg.channel,                              # channel
                      'Rx' if self.is_rx else 'Tx',             # direction
                      'R' if msg.is_remote_frame else 'D',      # remote/data
                      '%08Xh' % msg.arbitration_id,             # can id
                      msg.dlc,                                  # dlc
                      ' '.join('%02X' % x for x in msg.data))   # can data bytes
        if self.is_waterfall_mode:
            items = self.tvw_trace.get_children()
            if len(items) >= self.maxsize:
                self.tvw_trace.delete(items[-1])
            trace_data = ('%.4f' % ts,) + trace_data
            if msg.is_error_frame:
                self.tvw_trace.insert('', 0, values=trace_data,
                                      tags=('errframe',))
            else:
                self.tvw_trace.insert('', 0, values=trace_data)
        else:
            if msg.arbitration_id in self.canlist.keys():
                item = self.canlist[msg.arbitration_id][0]
                count = self.canlist[msg.arbitration_id][1] + 1
                self.canlist[msg.arbitration_id][1] = count
                trace_data = (str(count),) + trace_data
                if msg.is_error_frame:
                    self.tvw_trace.item(item, values=trace_data,
                                        tags=('errframe',))
                else:
                    self.tvw_trace.item(item, values=trace_data)
            else:
                trace_data = ('1',) + trace_data
                if msg.is_error_frame:
                    self.tvw_trace.insert('', -1, values=trace_data,
                                          tags=('errframe',))
                else:
                    self.tvw_trace.insert('', -1, values=trace_data)
                items = self.tvw_trace.get_children()
                self.canlist[msg.arbitration_id] = [items[0], 1]
    def sort(self):
        pass

class Maingui():
    '''
        Args:
    '''
    def __init__(self):
        self.root = Tk()
        self.canqueue = queue.Queue(10)
        self.canbus =  CanBackstage(self.canqueue, self.insert_can_message)
        self.bgcolor = "white"
        self.title = "Canbus Emulator"

        self.can_interface = StringVar()
        self.can_channel = StringVar()
        self.can_bitrate = StringVar()
        self.can_depot = StringVar()

        self.cbx_interface = ttk.Combobox(self.root, width=7,
                                          textvariable=self.can_interface)
        self.cbx_channel = ttk.Combobox(self.root, width=14,
                                        textvariable=self.can_channel)
        self.cbx_bitrate = ttk.Combobox(self.root, width=8,
                                        textvariable=self.can_bitrate)
        self.cbx_depot = ttk.Combobox(self.root, width=16,
                                      textvariable=self.can_depot)
        self.btn_connect = Button(self.root, text ="Connect", width=10,
                                  command=self.connect_can_interface)
        self.btn_listen = Button(self.root, text ="Listen", width=8,
                                 command=self.listen_canbus_trace)
        self.btn_start = Button(self.root, text ="Start", width=8,
                                command=self.start_canbus_trace)
        self.tvw_trace_rx = CanTrace(self.root, is_waterfall_mode=False,
                                     is_rx=True)
        self.tvw_trace_tx = CanTrace(self.root, is_waterfall_mode=False,
                                     is_rx=False)

        self.params = self.load_parameters()
        self.start_timestamp = 0
        self.transmitter = {}
        self.is_start_send = False
        self.send_timer = threading.Timer(0.01, self.canbus_transmit_timer)
        self.control_list = []

    def start(self):
        self.root.config(background=self.bgcolor)
        self.root.title(self.title)
        #self.root.geometry(self.winsize)
        self.root.resizable(0, 0)

        self.cbx_interface.grid(row=1, column=0, padx=5, pady=5)
        self.cbx_interface["value"] = self.get_hardware_interface()
        self.cbx_interface.current(0)
        self.cbx_interface.bind("<<ComboboxSelected>>", self.set_can_channel)

        self.cbx_channel.grid(row=1, column=1, padx=5, pady=5)
        self.cbx_channel["value"] = self.get_hardware_channel()[0]
        self.cbx_channel.current(0)

        self.cbx_bitrate.grid(row=1, column=2, padx=5, pady=5)
        self.cbx_bitrate["value"] = ("250000", "500000")
        self.cbx_bitrate.current(0)

        self.cbx_depot.grid(row=1, column=3, padx=5, pady=5)
        self.cbx_depot["value"] = self.get_depot_names()
        self.cbx_depot.current(0)
        self.cbx_depot.bind("<<ComboboxSelected>>", self.set_can_depot_layout)

        self.btn_connect.grid(row=1, column=4, padx=5, pady=5)
        self.btn_listen.grid(row=1, column=5, padx=5, pady=5)
        self.btn_start.grid(row=1, column=6, padx=5, pady=5)

        self.tvw_trace_rx.init(row=2, col=0)
        self.tvw_trace_tx.init(row=2, col=8)

        self.set_can_depot_layout(None)

        self.canbus.setDaemon(True)
        self.canbus.start()
        self.send_timer.start()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        self.is_start_send = True
        self.canqueue.put({"cmd": "exit"})
        self.send_timer.cancel()
        self.root.destroy()

    def load_parameters(self):
        parameters_dirpath = os.path.dirname(os.path.abspath(__file__))
        parameters_path = os.path.join(parameters_dirpath, 'can_matrix.json')
        print("Now loading: ", parameters_path)
        with open(parameters_path, 'r', encoding='utf8') as fp:
            parameters = json.load(fp)
        return parameters

    def get_hardware_interface(self):
        return tuple(x['interface'] for x in self.params["hardware"])

    def get_hardware_channel(self):
        return tuple(x['channel'] for x in self.params["hardware"])

    def get_depot_names(self):
        return tuple(x['name'] for x in self.params['depot'])

    def set_can_channel(self, event):
        for hal in self.params["hardware"]:
            if self.can_interface.get() == hal['interface']:
                self.cbx_channel["value"] = tuple(hal['channel'])
                self.cbx_channel.current(0)
                return
        raise ValueError("Unknow CAN interface: ", self.can_interface)

    def reset_can_transmitter(self):
        is_reset_can_send_timer = False
        depot = self.params['depot'][self.cbx_depot.current()]
        cnt = 0
        assert depot['name'] == self.can_depot.get()
        if self.is_start_send:
            is_reset_can_send_timer = True
            self.is_start_send = False

        self.transmitter = {}
        for key, value in depot['rx'].items():
            new_key = int(key, 16)
            self.transmitter[new_key] = {
                'counter': cnt,
                'period': value['period'],
                'msg': can.Message(arbitration_id=new_key,
                                   data=[0, 0, 0, 0, 0, 0, 0, 0]),
                'lock': threading.Lock()
            }
            cnt += 1
        if is_reset_can_send_timer:
            self.is_start_send = True

    def create_control_list(self):
        for ctl in self.control_list:
            ctl.destroy()
        self.control_list = []
        depot = self.params['depot'][self.cbx_depot.current()]
        assert depot['name'] == self.can_depot.get()
        cnt = 0
        for key, value in depot['rx'].items():
            for field_name, detail in value['field'].items():
                ft = FieldTable(self.root, field_name, int(key, 16), detail,
                                self.transmitter)
                row, col = 8 + cnt // 4, 3 * (cnt % 4)
                ft.canvas.grid(row=row, column=col, columnspan=3, padx=5)
                self.control_list.append(ft)
                cnt += 1

    def set_can_depot_layout(self, event):
        # Reset depot CAN transmitter
        self.reset_can_transmitter()
        # Reset depot CAN layout
        self.create_control_list()

    def connect_can_interface(self):
        if self.btn_connect["text"] == 'Disconnect':
            if self.btn_start["text"] == 'Stop':
                self.canqueue.put({"cmd": "stop_listen"})
                self.btn_start["bg"] = 'SystemButtonFace'
                self.btn_start["text"] = 'Start'
            self.canqueue.put({"cmd": "shutdown"})
            self.btn_connect["bg"] = 'SystemButtonFace'
            self.btn_connect["text"] = 'Connect'
        else:
            self.start_timestamp = time.time()
            self.canqueue.put({"cmd": "reset",
                                "interface": self.can_interface.get(),
                                "channel": self.can_channel.get(),
                                "bitrate": self.can_bitrate.get()})
            self.canqueue.put({"cmd": "connect"})
            self.btn_connect["bg"] = 'red'
            self.btn_connect["text"] = 'Disconnect'

    def listen_canbus_trace(self):
        if self.btn_connect["text"] == 'Disconnect':
            if self.btn_listen["text"] == 'Listening':
                self.canqueue.put({"cmd": "stop_listen"})
                self.btn_listen["bg"] = 'SystemButtonFace'
                self.btn_listen["text"] = 'Listen'
            else:
                self.canqueue.put({"cmd": "start_listen"})
                self.btn_listen["bg"] = 'red'
                self.btn_listen["text"] = 'Listening'

    def start_canbus_trace(self):
        if self.btn_connect["text"] == 'Disconnect':
            if self.btn_start["text"] == 'Stop':
                self.btn_start["bg"] = 'SystemButtonFace'
                self.btn_start["text"] = 'Start'
                self.is_start_send = False
            else:
                self.btn_start["bg"] = 'red'
                self.btn_start["text"] = 'Stop'
                self.is_start_send = True

    def canbus_transmit_timer(self):
        self.send_timer = threading.Timer(0.01, self.canbus_transmit_timer)
        if self.is_start_send:
            for key, value in self.transmitter.items():
                value['lock'].acquire()
                value['counter'] += 10
                if value['counter'] >= value['period']:
                    value['counter'] = 0
                    self.canqueue.put({"cmd": "send", "msg": value['msg']})
                value['lock'].release()
        self.send_timer.start()

    def insert_can_message(self, msg, is_rx):
        timestamp = time.time() - self.start_timestamp
        if is_rx:
            tvw_trace = self.tvw_trace_rx
        else:
            tvw_trace = self.tvw_trace_tx
        tvw_trace.update(timestamp, msg)

if __name__ == '__main__':
    main = Maingui()
    main.start()
